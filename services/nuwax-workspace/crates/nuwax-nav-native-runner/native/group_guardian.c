#define _GNU_SOURCE

#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#ifndef SYS_pidfd_open
#error "pidfd_open is required for native process-group supervision"
#endif

#ifndef SYS_pidfd_send_signal
#error "pidfd_send_signal is required for native process-group supervision"
#endif

#define GUARDIAN_PROTOCOL_BYTES 48U
#define GUARDIAN_PROTOCOL_VERSION 1U
#define GUARDIAN_MESSAGE_SPAWNED 1U
#define GUARDIAN_MESSAGE_READY 2U
#define GUARDIAN_MESSAGE_ERROR 3U
#define PAYLOAD_STACK_BYTES (1024U * 1024U)
#define CONTROL_DISARM ((unsigned char)'D')
#define PAYLOAD_RELEASE ((unsigned char)'R')

static const unsigned char GUARDIAN_MAGIC[8] = {'N', 'U', 'W', 'A', 'X', 'G', 'G', '1'};

struct payload_setup {
    int32_t error_number;
    int32_t process_group;
};

struct payload_args {
    char **argv;
    pid_t expected_parent;
    int setup_write_fd;
    int gate_read_fd;
    int setup_read_fd;
    int gate_write_fd;
    int parent_pidfd;
#ifdef NUWAX_GUARDIAN_TEST_HOOKS
    int test_stage;
    const char *test_state_path;
#endif
};

#ifdef NUWAX_GUARDIAN_TEST_HOOKS
enum guardian_test_stage {
    TEST_STAGE_NONE = 0,
    TEST_STAGE_CLONED = 1,
    TEST_STAGE_BEFORE_SETPGID = 2,
    TEST_STAGE_AFTER_SETPGID = 3,
    TEST_STAGE_BEFORE_EXEC = 4,
};
static int test_forced_exit_code = 0;
#endif

static void close_preserving_errno(int fd) {
    int saved = errno;
    if (fd >= 0) {
        (void)close(fd);
    }
    errno = saved;
}

static int write_full(int fd, const void *buffer, size_t length) {
    const unsigned char *cursor = buffer;
    size_t remaining = length;
    while (remaining > 0U) {
        ssize_t written = write(fd, cursor, remaining);
        if (written > 0) {
            size_t amount = (size_t)written;
            cursor += amount;
            remaining -= amount;
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        return -1;
    }
    return 0;
}

static int read_full(int fd, void *buffer, size_t length) {
    unsigned char *cursor = buffer;
    size_t remaining = length;
    while (remaining > 0U) {
        ssize_t received = read(fd, cursor, remaining);
        if (received > 0) {
            size_t amount = (size_t)received;
            cursor += amount;
            remaining -= amount;
            continue;
        }
        if (received < 0 && errno == EINTR) {
            continue;
        }
        if (received == 0) {
            errno = EPIPE;
        }
        return -1;
    }
    return 0;
}

static void encode_u32(unsigned char *destination, uint32_t value) {
    uint32_t encoded = htonl(value);
    memcpy(destination, &encoded, sizeof(encoded));
}

static int send_guardian_message(uint32_t kind, int error_number, pid_t guardian_pid,
                                 pid_t parent_pid, pid_t payload_pid, pid_t process_group,
                                 int parent_pidfd_ready, int payload_pidfd_ready) {
    unsigned char message[GUARDIAN_PROTOCOL_BYTES];
    memset(message, 0, sizeof(message));
    memcpy(message, GUARDIAN_MAGIC, sizeof(GUARDIAN_MAGIC));
    encode_u32(message + 8U, GUARDIAN_PROTOCOL_VERSION);
    encode_u32(message + 12U, kind);
    encode_u32(message + 16U, (uint32_t)error_number);
    encode_u32(message + 20U, (uint32_t)guardian_pid);
    encode_u32(message + 24U, (uint32_t)parent_pid);
    encode_u32(message + 28U, (uint32_t)payload_pid);
    encode_u32(message + 32U, (uint32_t)process_group);
    encode_u32(message + 36U, (uint32_t)parent_pidfd_ready);
    encode_u32(message + 40U, (uint32_t)payload_pidfd_ready);
    return write_full(STDOUT_FILENO, message, sizeof(message));
}

static int pidfd_open_exact(pid_t pid) {
    return (int)syscall(SYS_pidfd_open, pid, 0U);
}

static int pidfd_signal_exact(int pidfd, int signal_number) {
    return (int)syscall(SYS_pidfd_send_signal, pidfd, signal_number, NULL, 0U);
}

static int pidfd_is_readable(int pidfd) {
    struct pollfd descriptor = {.fd = pidfd, .events = POLLIN, .revents = 0};
    int result;
    do {
        result = poll(&descriptor, 1U, 0);
    } while (result < 0 && errno == EINTR);
    if (result < 0) {
        return -1;
    }
    return result > 0 && (descriptor.revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) != 0;
}

static int parse_positive(const char *text, long maximum, long *value) {
    char *end = NULL;
    errno = 0;
    long parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed <= 0 || parsed > maximum) {
        errno = EINVAL;
        return -1;
    }
    *value = parsed;
    return 0;
}

static int monotonic_millis(int64_t *value) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return -1;
    }
    *value = ((int64_t)now.tv_sec * 1000) + ((int64_t)now.tv_nsec / 1000000);
    return 0;
}

static void report_payload_setup(int fd, int error_number, pid_t process_group) {
    struct payload_setup setup = {
        .error_number = (int32_t)error_number,
        .process_group = (int32_t)process_group,
    };
    (void)write_full(fd, &setup, sizeof(setup));
}

#ifdef NUWAX_GUARDIAN_TEST_HOOKS
static int parse_test_stage(const char *text, int *stage) {
    long parsed = 0;
    if (parse_positive(text, TEST_STAGE_BEFORE_EXEC, &parsed) != 0) {
        return -1;
    }
    *stage = (int)parsed;
    return 0;
}

static void test_checkpoint(const struct payload_args *arguments, int stage) {
    if (arguments->test_stage != stage) {
        return;
    }
    if (arguments->test_state_path == NULL || arguments->test_state_path[0] != '/') {
        _exit(124);
    }
    int state_fd = open(arguments->test_state_path,
                        O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (state_fd < 0) {
        _exit(124);
    }
    char state[96];
    int length = snprintf(state, sizeof(state), "%d %ld %ld\n", stage, (long)getpid(),
                          (long)getpgrp());
    if (length <= 0 || (size_t)length >= sizeof(state) ||
        write_full(state_fd, state, (size_t)length) != 0) {
        close_preserving_errno(state_fd);
        _exit(124);
    }
    close_preserving_errno(state_fd);
    if (raise(SIGSTOP) != 0) {
        _exit(124);
    }
}
#else
static void test_checkpoint(const struct payload_args *arguments, int stage) {
    (void)arguments;
    (void)stage;
}
#define TEST_STAGE_CLONED 0
#define TEST_STAGE_BEFORE_SETPGID 0
#define TEST_STAGE_AFTER_SETPGID 0
#define TEST_STAGE_BEFORE_EXEC 0
#endif

static int payload_main(void *opaque) {
    struct payload_args *arguments = opaque;
    close_preserving_errno(arguments->setup_read_fd);
    close_preserving_errno(arguments->gate_write_fd);

    test_checkpoint(arguments, TEST_STAGE_CLONED);

    if (prctl(PR_SET_PDEATHSIG, SIGSTOP) != 0) {
        report_payload_setup(arguments->setup_write_fd, errno, 0);
        _exit(125);
    }
    if (getppid() != arguments->expected_parent) {
        report_payload_setup(arguments->setup_write_fd, EPIPE, 0);
        _exit(125);
    }
    test_checkpoint(arguments, TEST_STAGE_BEFORE_SETPGID);
    if (setpgid(0, 0) != 0) {
        report_payload_setup(arguments->setup_write_fd, errno, 0);
        _exit(125);
    }
    test_checkpoint(arguments, TEST_STAGE_AFTER_SETPGID);

    int null_fd = open("/dev/null", O_RDONLY | O_CLOEXEC);
    if (null_fd < 0 || dup2(null_fd, STDIN_FILENO) < 0) {
        int failure = errno;
        close_preserving_errno(null_fd);
        report_payload_setup(arguments->setup_write_fd, failure, 0);
        _exit(125);
    }
    close_preserving_errno(null_fd);
    close_preserving_errno(arguments->parent_pidfd);

    pid_t payload_pid = getpid();
    report_payload_setup(arguments->setup_write_fd, 0, payload_pid);
    close_preserving_errno(arguments->setup_write_fd);

    unsigned char release = 0U;
    if (read_full(arguments->gate_read_fd, &release, sizeof(release)) != 0 ||
        release != PAYLOAD_RELEASE) {
        _exit(125);
    }
    close_preserving_errno(arguments->gate_read_fd);

    test_checkpoint(arguments, TEST_STAGE_BEFORE_EXEC);
    struct sigaction default_sigpipe;
    memset(&default_sigpipe, 0, sizeof(default_sigpipe));
    default_sigpipe.sa_handler = SIG_DFL;
    (void)sigemptyset(&default_sigpipe.sa_mask);
    if (sigaction(SIGPIPE, &default_sigpipe, NULL) != 0) {
        dprintf(STDERR_FILENO, "native group guardian failed to restore SIGPIPE: %s\n",
                strerror(errno));
        _exit(125);
    }
    execvp(arguments->argv[0], arguments->argv);
    int failure = errno;
    dprintf(STDERR_FILENO, "native group guardian failed to exec payload: %s\n",
            strerror(failure));
    _exit(failure == ENOENT ? 127 : 126);
}

static int process_group_exists(pid_t process_group) {
    if (kill(-process_group, 0) == 0 || errno == EPERM) {
        return 1;
    }
    if (errno == ESRCH) {
        return 0;
    }
    return -1;
}

static int inspect_nonleader_members(pid_t process_group, pid_t payload_pid, int signal_number,
                                     int *live_members, int *zombie_members) {
    DIR *proc = opendir("/proc");
    if (proc == NULL) {
        return -1;
    }
    *live_members = 0;
    *zombie_members = 0;
    for (;;) {
        errno = 0;
        struct dirent *entry = readdir(proc);
        if (entry == NULL) {
            break;
        }
        char *end = NULL;
        errno = 0;
        long parsed = strtol(entry->d_name, &end, 10);
        if (errno != 0 || end == entry->d_name || *end != '\0' || parsed <= 0 ||
            parsed > INT32_MAX || (pid_t)parsed == payload_pid) {
            continue;
        }
        pid_t member_pid = (pid_t)parsed;
        pid_t member_group = getpgid(member_pid);
        if (member_group < 0) {
            if (errno == ESRCH) {
                continue;
            }
            int failure = errno;
            (void)closedir(proc);
            errno = failure;
            return -1;
        }
        if (member_group != process_group) {
            continue;
        }
        int member_pidfd = pidfd_open_exact(member_pid);
        if (member_pidfd < 0) {
            if (errno == ESRCH) {
                continue;
            }
            int failure = errno;
            (void)closedir(proc);
            errno = failure;
            return -1;
        }
        int terminal = pidfd_is_readable(member_pidfd);
        if (terminal < 0) {
            int failure = errno;
            close_preserving_errno(member_pidfd);
            (void)closedir(proc);
            errno = failure;
            return -1;
        }
        if (terminal == 1) {
            *zombie_members += 1;
            close_preserving_errno(member_pidfd);
            continue;
        }
        *live_members += 1;
        if (signal_number == 0) {
            close_preserving_errno(member_pidfd);
            continue;
        }
        if (pidfd_signal_exact(member_pidfd, signal_number) != 0 && errno != ESRCH) {
            int failure = errno;
            close_preserving_errno(member_pidfd);
            (void)closedir(proc);
            errno = failure;
            return -1;
        }
        close_preserving_errno(member_pidfd);
    }
    int iteration_failure = errno;
    if (closedir(proc) != 0 && iteration_failure == 0) {
        return -1;
    }
    if (iteration_failure != 0) {
        errno = iteration_failure;
        return -1;
    }
    return 0;
}

static int wait_for_live_nonleaders(pid_t process_group, pid_t payload_pid, long timeout_ms) {
    int64_t started = 0;
    if (monotonic_millis(&started) != 0) {
        return -1;
    }
    for (;;) {
        int live_members = 0;
        int zombie_members = 0;
        if (inspect_nonleader_members(process_group, payload_pid, 0, &live_members,
                                      &zombie_members) != 0) {
            return -1;
        }
        if (live_members == 0) {
            (void)zombie_members;
            return 1;
        }
        int64_t now = 0;
        if (monotonic_millis(&now) != 0) {
            return -1;
        }
        if (now - started >= timeout_ms) {
            return 0;
        }
        struct timespec pause = {.tv_sec = 0, .tv_nsec = 5000000L};
        while (nanosleep(&pause, &pause) != 0 && errno == EINTR) {
        }
    }
}

static int clean_process_group(pid_t payload_pid, pid_t process_group, int payload_pidfd,
                               long cleanup_timeout_ms) {
    if (process_group <= 0 || payload_pid <= 0) {
        return 0;
    }
    int owned_pidfd = -1;
    if (payload_pidfd < 0) {
        owned_pidfd = pidfd_open_exact(payload_pid);
        if (owned_pidfd < 0) {
            int pidfd_failure = errno;
            int group_exists = process_group_exists(process_group);
            if (pidfd_failure == ESRCH && group_exists == 0) {
                return 0;
            }
            errno = pidfd_failure;
            return -1;
        }
        payload_pidfd = owned_pidfd;
    }
    int leader_terminal = pidfd_is_readable(payload_pidfd);
    int group_established = leader_terminal == 1;
    if (leader_terminal < 0) {
        close_preserving_errno(owned_pidfd);
        return -1;
    }
    if (leader_terminal == 0) {
        if (pidfd_signal_exact(payload_pidfd, SIGSTOP) != 0 && errno != ESRCH) {
            close_preserving_errno(owned_pidfd);
            return -1;
        }
        pid_t observed_group = getpgid(payload_pid);
        if (observed_group == process_group) {
            group_established = 1;
            if (kill(-process_group, SIGSTOP) != 0 && errno != ESRCH) {
                close_preserving_errno(owned_pidfd);
                return -1;
            }
        } else if (observed_group < 0 && errno != ESRCH) {
            close_preserving_errno(owned_pidfd);
            return -1;
        }
    }
    int live_members = 0;
    int zombie_members = 0;
    if (group_established != 0 &&
        inspect_nonleader_members(process_group, payload_pid, SIGTERM, &live_members,
                                  &zombie_members) != 0) {
        close_preserving_errno(owned_pidfd);
        return -1;
    }
    long term_timeout_ms = cleanup_timeout_ms / 2;
    if (term_timeout_ms < 1) {
        term_timeout_ms = 1;
    }
    int cleaned = group_established != 0
                      ? wait_for_live_nonleaders(process_group, payload_pid, term_timeout_ms)
                      : 1;
    if (cleaned < 0) {
        close_preserving_errno(owned_pidfd);
        return -1;
    }
    if (cleaned == 0 &&
        inspect_nonleader_members(process_group, payload_pid, SIGKILL, &live_members,
                                  &zombie_members) != 0) {
        close_preserving_errno(owned_pidfd);
        return -1;
    }
    if (cleaned == 0) {
        cleaned = wait_for_live_nonleaders(process_group, payload_pid, cleanup_timeout_ms);
    }
    if (cleaned != 1) {
        if (cleaned == 0) {
            errno = ETIMEDOUT;
        }
        close_preserving_errno(owned_pidfd);
        return -1;
    }
    if (pidfd_is_readable(payload_pidfd) == 0 &&
        pidfd_signal_exact(payload_pidfd, SIGKILL) != 0 && errno != ESRCH) {
        close_preserving_errno(owned_pidfd);
        return -1;
    }
    int64_t leader_started = 0;
    if (monotonic_millis(&leader_started) != 0) {
        close_preserving_errno(owned_pidfd);
        return -1;
    }
    for (;;) {
        leader_terminal = pidfd_is_readable(payload_pidfd);
        if (leader_terminal == 1) {
            break;
        }
        if (leader_terminal < 0) {
            close_preserving_errno(owned_pidfd);
            return -1;
        }
        int64_t now = 0;
        if (monotonic_millis(&now) != 0 || now - leader_started >= cleanup_timeout_ms) {
            close_preserving_errno(owned_pidfd);
            errno = ETIMEDOUT;
            return -1;
        }
        struct timespec pause = {.tv_sec = 0, .tv_nsec = 5000000L};
        while (nanosleep(&pause, &pause) != 0 && errno == EINTR) {
        }
    }
    close_preserving_errno(owned_pidfd);
    return 0;
}

static int parent_is_alive(pid_t expected_parent, int parent_pidfd) {
    int readable = pidfd_is_readable(parent_pidfd);
    return readable == 0 && getppid() == expected_parent;
}

static int monitor_parent(pid_t expected_parent, int parent_pidfd, int payload_pidfd) {
    struct pollfd descriptors[2] = {
        {.fd = parent_pidfd, .events = POLLIN, .revents = 0},
        {.fd = STDIN_FILENO, .events = POLLIN | POLLHUP, .revents = 0},
    };
    for (;;) {
        descriptors[0].revents = 0;
        descriptors[1].revents = 0;
        int result;
        do {
            result = poll(descriptors, 2U, -1);
        } while (result < 0 && errno == EINTR);
        if (result < 0) {
            return -1;
        }
#ifdef NUWAX_GUARDIAN_TEST_HOOKS
        if (test_forced_exit_code == 74) {
            errno = EIO;
            return -1;
        }
#endif
        if ((descriptors[0].revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) != 0) {
            return 0;
        }
        if ((descriptors[1].revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
            return 0;
        }
        if ((descriptors[1].revents & POLLIN) != 0) {
            unsigned char command = 0U;
            ssize_t received;
            do {
                received = read(STDIN_FILENO, &command, sizeof(command));
            } while (received < 0 && errno == EINTR);
            if (received != 1 || command != CONTROL_DISARM) {
                return 0;
            }
            if (!parent_is_alive(expected_parent, parent_pidfd)) {
                return 0;
            }
            if (pidfd_is_readable(payload_pidfd) != 1) {
                errno = EBUSY;
                return -1;
            }
            return 1;
        }
    }
}

int main(int argc, char **argv) {
    struct sigaction ignore_sigpipe;
    memset(&ignore_sigpipe, 0, sizeof(ignore_sigpipe));
    ignore_sigpipe.sa_handler = SIG_IGN;
    (void)sigemptyset(&ignore_sigpipe.sa_mask);
    if (sigaction(SIGPIPE, &ignore_sigpipe, NULL) != 0) {
        return 70;
    }
    int payload_argument = 6;
#ifdef NUWAX_GUARDIAN_TEST_HOOKS
    int test_stage = TEST_STAGE_NONE;
    const char *test_state_path = NULL;
    if (argc >= 11 && strcmp(argv[5], "--test-stage") == 0 &&
        strcmp(argv[7], "--test-state-path") == 0 && strcmp(argv[9], "--") == 0) {
        payload_argument = 10;
        test_state_path = argv[8];
        if (parse_test_stage(argv[6], &test_stage) != 0 || test_state_path[0] != '/') {
            dprintf(STDERR_FILENO, "invalid native group guardian test arguments\n");
            return 64;
        }
    } else if (argc >= 9 && strcmp(argv[5], "--test-exit-code") == 0 &&
               strcmp(argv[7], "--") == 0) {
        long parsed_exit = 0;
        payload_argument = 8;
        if (parse_positive(argv[6], 75, &parsed_exit) != 0 ||
            (parsed_exit != 73 && parsed_exit != 74 && parsed_exit != 75)) {
            dprintf(STDERR_FILENO, "invalid native group guardian forced exit\n");
            return 64;
        }
        test_forced_exit_code = (int)parsed_exit;
    }
#endif
    if (argc <= payload_argument || strcmp(argv[1], "--parent-pid") != 0 ||
        strcmp(argv[3], "--cleanup-timeout-ms") != 0 ||
        strcmp(argv[payload_argument - 1], "--") != 0) {
        dprintf(STDERR_FILENO, "invalid native group guardian arguments\n");
        return 64;
    }

    long parent_value = 0;
    long timeout_value = 0;
    if (parse_positive(argv[2], INT32_MAX, &parent_value) != 0 ||
        parse_positive(argv[4], INT32_MAX, &timeout_value) != 0) {
        dprintf(STDERR_FILENO, "invalid native group guardian numeric argument\n");
        return 64;
    }
    pid_t expected_parent = (pid_t)parent_value;
    pid_t guardian_pid = getpid();
    if (getppid() != expected_parent) {
        dprintf(STDERR_FILENO, "native group guardian parent changed before setup\n");
        return 65;
    }

    int parent_pidfd = pidfd_open_exact(expected_parent);
    if (parent_pidfd < 0 || !parent_is_alive(expected_parent, parent_pidfd)) {
        dprintf(STDERR_FILENO, "native group guardian could not pin its parent: %s\n",
                strerror(errno));
        close_preserving_errno(parent_pidfd);
        return 65;
    }

    int setup_pipe[2] = {-1, -1};
    int gate_pipe[2] = {-1, -1};
    if (pipe2(setup_pipe, O_CLOEXEC) != 0 || pipe2(gate_pipe, O_CLOEXEC) != 0) {
        int failure = errno;
        (void)send_guardian_message(GUARDIAN_MESSAGE_ERROR, failure, guardian_pid,
                                    expected_parent, 0, 0, 1, 0);
        close_preserving_errno(setup_pipe[0]);
        close_preserving_errno(setup_pipe[1]);
        close_preserving_errno(gate_pipe[0]);
        close_preserving_errno(gate_pipe[1]);
        close_preserving_errno(parent_pidfd);
        return 66;
    }

    void *stack = malloc(PAYLOAD_STACK_BYTES);
    if (stack == NULL) {
        int failure = errno == 0 ? ENOMEM : errno;
        (void)send_guardian_message(GUARDIAN_MESSAGE_ERROR, failure, guardian_pid,
                                    expected_parent, 0, 0, 1, 0);
        return 66;
    }
    struct payload_args payload = {
        .argv = &argv[payload_argument],
        .expected_parent = expected_parent,
        .setup_write_fd = setup_pipe[1],
        .gate_read_fd = gate_pipe[0],
        .setup_read_fd = setup_pipe[0],
        .gate_write_fd = gate_pipe[1],
        .parent_pidfd = parent_pidfd,
#ifdef NUWAX_GUARDIAN_TEST_HOOKS
        .test_stage = test_stage,
        .test_state_path = test_state_path,
#endif
    };
    unsigned char *stack_top = (unsigned char *)stack + PAYLOAD_STACK_BYTES;
    int clone_flags = CLONE_PARENT | SIGCHLD;
    pid_t payload_pid = clone(payload_main, stack_top, clone_flags, &payload);
    if (payload_pid < 0) {
        int failure = errno;
        (void)send_guardian_message(GUARDIAN_MESSAGE_ERROR, failure, guardian_pid,
                                    expected_parent, 0, 0, 1, 0);
        free(stack);
        return 67;
    }
    close_preserving_errno(setup_pipe[1]);
    close_preserving_errno(gate_pipe[0]);

    if (send_guardian_message(GUARDIAN_MESSAGE_SPAWNED, 0, guardian_pid, expected_parent,
                              payload_pid, payload_pid, 1, 0) != 0) {
        close_preserving_errno(gate_pipe[1]);
        (void)clean_process_group(payload_pid, payload_pid, -1, timeout_value);
        free(stack);
        return 68;
    }

    struct pollfd setup_descriptors[3] = {
        {.fd = parent_pidfd, .events = POLLIN, .revents = 0},
        {.fd = STDIN_FILENO, .events = POLLIN | POLLHUP, .revents = 0},
        {.fd = setup_pipe[0], .events = POLLIN | POLLHUP, .revents = 0},
    };
    int setup_poll;
    do {
        setup_poll = poll(setup_descriptors, 3U, (int)timeout_value);
    } while (setup_poll < 0 && errno == EINTR);
    if (setup_poll <= 0 ||
        (setup_descriptors[0].revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) != 0 ||
        (setup_descriptors[1].revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        close_preserving_errno(gate_pipe[1]);
        (void)clean_process_group(payload_pid, payload_pid, -1, timeout_value);
        free(stack);
        return 69;
    }

    struct payload_setup setup = {.error_number = EPROTO, .process_group = 0};
    if (read_full(setup_pipe[0], &setup, sizeof(setup)) != 0 || setup.error_number != 0 ||
        setup.process_group != payload_pid) {
        int failure = setup.error_number != 0 ? setup.error_number : EPROTO;
        (void)send_guardian_message(GUARDIAN_MESSAGE_ERROR, failure, guardian_pid,
                                    expected_parent, payload_pid, payload_pid, 1, 0);
        close_preserving_errno(gate_pipe[1]);
        (void)clean_process_group(payload_pid, payload_pid, -1, timeout_value);
        free(stack);
        return 70;
    }
    close_preserving_errno(setup_pipe[0]);

    int payload_pidfd = pidfd_open_exact(payload_pid);
    if (payload_pidfd < 0 || getpgid(payload_pid) != payload_pid) {
        int failure = errno == 0 ? EPROTO : errno;
        (void)send_guardian_message(GUARDIAN_MESSAGE_ERROR, failure, guardian_pid,
                                    expected_parent, payload_pid, payload_pid, 1, 0);
        close_preserving_errno(gate_pipe[1]);
        (void)clean_process_group(payload_pid, payload_pid, payload_pidfd, timeout_value);
        free(stack);
        return 71;
    }

#ifdef NUWAX_GUARDIAN_TEST_HOOKS
    if (test_forced_exit_code != 0) {
        close_preserving_errno(gate_pipe[1]);
        int cleanup = clean_process_group(payload_pid, payload_pid, payload_pidfd, timeout_value);
        close_preserving_errno(payload_pidfd);
        close_preserving_errno(parent_pidfd);
        free(stack);
        if (cleanup != 0) {
            return 73;
        }
        if (test_forced_exit_code == 75) {
            (void)raise(SIGKILL);
            return 75;
        }
        return test_forced_exit_code;
    }
#endif

    if (send_guardian_message(GUARDIAN_MESSAGE_READY, 0, guardian_pid, expected_parent,
                              payload_pid, payload_pid, 1, 1) != 0 ||
        write_full(gate_pipe[1], &(unsigned char){PAYLOAD_RELEASE}, 1U) != 0) {
        close_preserving_errno(gate_pipe[1]);
        (void)clean_process_group(payload_pid, payload_pid, payload_pidfd, timeout_value);
        free(stack);
        return 72;
    }
    close_preserving_errno(gate_pipe[1]);
    free(stack);

    int disposition = monitor_parent(expected_parent, parent_pidfd, payload_pidfd);
    if (disposition == 1) {
        close_preserving_errno(payload_pidfd);
        close_preserving_errno(parent_pidfd);
        return 0;
    }
    if (clean_process_group(payload_pid, payload_pid, payload_pidfd, timeout_value) != 0) {
        dprintf(STDERR_FILENO, "native group guardian cleanup proof failed: %s\n",
                strerror(errno));
        close_preserving_errno(payload_pidfd);
        close_preserving_errno(parent_pidfd);
        return 73;
    }
    close_preserving_errno(payload_pidfd);
    close_preserving_errno(parent_pidfd);
    return disposition == 0 ? 0 : 74;
}
