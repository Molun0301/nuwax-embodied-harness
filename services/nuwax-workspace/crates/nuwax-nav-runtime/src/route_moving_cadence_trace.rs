//! Bounded diagnostics for the opt-in moving fixture, never deadline authority.
use super::{Probe, RawReceipt, MAX_TICKS};
use std::{fs::File, io::{Read, Seek, SeekFrom}, time::{Duration, Instant}};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Counters { runtime_ns:u64, runqueue_ns:u64, timeslices:u64 }
fn parse(bytes:&[u8])->Option<Counters> {
    let mut fields=bytes.split(u8::is_ascii_whitespace).filter(|field|!field.is_empty());
    let mut number=||fields.next()?.iter().try_fold(0_u64,|n,&b| {
        b.is_ascii_digit().then_some(())?;
        n.checked_mul(10)?.checked_add(u64::from(b-b'0'))
    });
    let result=Counters{runtime_ns:number()?,runqueue_ns:number()?,timeslices:number()?};
    if fields.next().is_some(){None}else{Some(result)}
}
#[derive(Clone, Copy, Debug)]
pub(super) struct SchedRead {
    tid:Option<u32>, start:Instant, end:Instant, counters:Result<Counters,&'static str>,
}
pub(super) struct SchedReader { file:Option<File>, tid:Option<u32>, buffer:[u8;128] }
impl SchedReader {
    // Must be called on the executing thread, never the planner constructor.
    pub(super) fn open()->Self {
        let tid=std::fs::read_link("/proc/thread-self").ok()
            .and_then(|p|p.file_name()?.to_str()?.parse().ok());
        Self{file:File::open("/proc/thread-self/schedstat").ok(),tid,buffer:[0;128]}
    }
    pub(super) fn sample(&mut self)->SchedRead {
        let start=Instant::now();
        let counters=(|| {
            let file=self.file.as_mut().ok_or("unavailable_open")?;
            file.seek(SeekFrom::Start(0)).map_err(|_|"seek_failed")?;
            let mut used=0;
            loop {
                if used==self.buffer.len(){return Err("truncated_or_oversize");}
                let n=file.read(&mut self.buffer[used..]).map_err(|_|"read_failed")?;
                if n==0{break;}used+=n;
            }
            let counters=parse(&self.buffer[..used]).ok_or("invalid_fields")?;
            if counters==(Counters{runtime_ns:0,runqueue_ns:0,timeslices:0}) {return Err("zero_triple_disabled_or_uninitialized");}
            Ok(counters)
        })();
        SchedRead{tid:self.tid,start,end:Instant::now(),counters}
    }
}
fn delta(a:SchedRead,b:SchedRead)->Result<Counters,&'static str> {
    if a.tid.is_none()||a.tid!=b.tid{return Err("missing_or_changed_thread_identity");}
    if a.end.checked_duration_since(a.start).is_none()||b.end.checked_duration_since(b.start).is_none()
        ||b.start.checked_duration_since(a.end).is_none(){return Err("invalid_read_window_order");}
    let (a,b)=(a.counters?,b.counters?);
    Ok(Counters{runtime_ns:b.runtime_ns.checked_sub(a.runtime_ns).ok_or("runtime_nonmonotonic")?,
        runqueue_ns:b.runqueue_ns.checked_sub(a.runqueue_ns).ok_or("runqueue_nonmonotonic")?,
        timeslices:b.timeslices.checked_sub(a.timeslices).ok_or("timeslices_nonmonotonic")?})
}
fn ms(d:Duration)->f64 {d.as_secs_f64()*1e3}
fn span(a:Instant,b:Instant)->Option<f64> {b.checked_duration_since(a).map(ms)}
fn report_sched(label:&str,origin:Instant,a:SchedRead,b:SchedRead) {
    eprintln!("CADENCE_SCHED interval={label} tid_before={:?} tid_after={:?} before_window_ms={:?} after_window_ms={:?} read_cost_ms={:?} counters_before={:?} counters_after={:?} checked_delta={:?}",
        a.tid,b.tid,(span(origin,a.start),span(origin,a.end)),(span(origin,b.start),span(origin,b.end)),
        (span(a.start,a.end),span(b.start,b.end)),a.counters,b.counters,delta(a,b));
}

pub(super) struct Tick {
    pub tick:usize, pub incoming_deadline:Instant, pub before_sleep:Instant,
    pub requested_sleep:Option<Duration>, pub start:Instant, pub gap:Duration,
    // Ordered endpoints: integration, frame (includes nested map call), odom,
    // held-before, worker step, receipt/applied feedback, held-after, body end.
    pub ends:[Instant;8], pub outgoing_deadline:Option<Instant>, pub reset:bool,
    pub probe_wait:[Option<Duration>;3], pub map_call:Option<Duration>,
    pub sched_before_sleep:SchedRead, pub sched_after_wake:SchedRead, pub sched_body_end:SchedRead,
}
struct GapParts { previous_body:Duration, intertick:Duration, wait:Duration }
fn decompose(previous_start:Instant,previous_end:Instant,before_sleep:Instant,start:Instant)->Option<GapParts> {
    Some(GapParts{previous_body:previous_end.checked_duration_since(previous_start)?,
        intertick:before_sleep.checked_duration_since(previous_end)?,wait:start.checked_duration_since(before_sleep)?})
}
pub(super) struct Trace { pub ticks:Vec<Tick>, pub overflow:usize, limit:usize }
impl Trace {
    pub(super) fn new(max_ticks:usize)->Self {
        let limit=max_ticks.min(MAX_TICKS*2);Self{ticks:Vec::with_capacity(limit),overflow:0,limit}
    }
    pub(super) fn push(&mut self,tick:Tick) {
        if self.ticks.len()<self.limit{self.ticks.push(tick);}else{self.overflow=self.overflow.saturating_add(1);}
    }
    pub(super) fn report(&self,origin:Instant,receipts:&[RawReceipt],probe:&Probe,factory:bool) {
        eprintln!("CADENCE_TRACE retained={} limit={} overflow={} complete={} outlier_threshold_ms=75 raw_50ms_counts=MOVING_summary scalar_preallocated=true diagnostic_cost_inclusive=true scheduler_counters=accounted_runtime_ns_runqueue_ns_timeslices not_exact_high_resolution_thread_cpu=true zero_delta_does_not_prove_zero_work=true accounting_lag_not_bounded_by_read_window=true sensor_scheduler_attribution=unavailable existing_sensor_wall_stages=SOURCE_ASSEMBLY",
            self.ticks.len(),self.limit,self.overflow,self.overflow==0);
        eprintln!("CADENCE_INNER_LIMITS available={} control_over_50ms={} map_over_50ms={} control_samples_omitted={} raw_limits_unchanged=true",
            !factory,probe.control_ms.iter().filter(|&&v|v>50.0).count(),probe.map_update_ms.iter().filter(|&&v|v>50.0).count(),probe.control_sample_overflow);
        let consumer_reads=self.ticks.iter().flat_map(|t|[t.sched_before_sleep,t.sched_after_wake,t.sched_body_end]);
        let compute_reads=probe.control_samples.iter().flat_map(|s|[s.sched_before,s.sched_after]);
        for (owner,reads) in [("consumer",consumer_reads.collect::<Vec<_>>()),("compute",compute_reads.collect::<Vec<_>>())] {
            let costs:Vec<_>=reads.iter().filter_map(|s|span(s.start,s.end)).collect();
            eprintln!("CADENCE_SCHED_SCOPE owner={owner} reads={} invalid_or_missing={} missing_thread_identity={} read_cost_ms_median_p95_max={} initial_sample={:?} factory_unprobed={factory}",
                reads.len(),reads.iter().filter(|s|s.counters.is_err()).count(),reads.iter().filter(|s|s.tid.is_none()).count(),super::measured_distribution(&costs),reads.first());
        }
        for (label,values) in [
            ("profile_begin",probe.control_samples.iter().map(|s|ms(s.profile_begin)).collect::<Vec<_>>()),
            ("profile_end",probe.control_samples.iter().map(|s|ms(s.profile_end)).collect()),
            ("compute_probe_lock_wait",probe.control_samples.iter().map(|s|ms(s.probe_wait)).collect()),
            ("successful_map_probe_lock_wait",probe.source_installs.iter().map(|s|ms(s.probe_wait)).collect()),
        ] {eprintln!("CADENCE_DIAGNOSTIC_COST stage={label} ms_median_p95_max={} retained_samples={} inclusive_not_subtracted=true",super::measured_distribution(&values),values.len());}
        for pair in self.ticks.windows(2).filter(|pair|pair[1].gap>Duration::from_millis(75)) {
            let (previous,current)=(&pair[0],&pair[1]);
            let parts=decompose(previous.start,previous.ends[7],current.before_sleep,current.start);
            let gap_parts=parts.as_ref().map(|p|(ms(p.previous_body),ms(p.intertick),ms(p.wait)));
            let reconstructs=parts.as_ref().is_some_and(|p|p.previous_body+p.intertick+p.wait==current.gap);
            let sleep_window=current.start.checked_duration_since(current.before_sleep);
            let oversleep=current.requested_sleep.and_then(|requested|sleep_window?.checked_sub(requested)).map(ms);
            eprintln!("CADENCE_OUTLIER tick={} previous_tick={} raw_gap_ms={} previous_body_intertick_sleepwindow_ms={gap_parts:?} exact_gap_reconstruction={reconstructs} requested_sleep_ms={:?} checked_oversleep_ms={oversleep:?} no_sleep_requested={} no_adjusted_pass=true",
                current.tick,previous.tick,ms(current.gap),current.requested_sleep.map(ms),current.requested_sleep.is_none());
            for tick in [previous,current] {
                let mut prior=tick.start;
                let phases=tick.ends.map(|end|{let duration=span(prior,end);prior=end;duration});
                eprintln!("CADENCE_TICK tick={} incoming_deadline_ms={:?} before_sleep_ms={:?} requested_sleep_ms={:?} after_wake_original_tickstart_ms={:?} body_end_ms={:?} outgoing_deadline_ms={:?} deadline_reset={} phase_ms={phases:?} phase_order=integration_including_wake_read,frame_including_map,odom,held_before,worker_step,receipt_applied_feedback,held_after,tail frame_nested_map_call_ms={:?} probe_lock_wait_frame_holdstart_continuity_ms={:?} nested_values_do_not_sum=true",
                    tick.tick,span(origin,tick.incoming_deadline),span(origin,tick.before_sleep),tick.requested_sleep.map(ms),span(origin,tick.start),span(origin,tick.ends[7]),tick.outgoing_deadline.and_then(|t|span(origin,t)),tick.reset,tick.map_call.map(ms),tick.probe_wait.map(|v|v.map(ms)));
                report_sched("consumer_before_sleep_to_after_wake",origin,tick.sched_before_sleep,tick.sched_after_wake);
                report_sched("consumer_after_wake_to_body_end",origin,tick.sched_after_wake,tick.sched_body_end);
                let receipt=receipts.iter().find(|receipt|receipt.tick==tick.tick);
                if let Some(receipt)=receipt {
                    if let Some(context)=&receipt.proof_context {
                        let proof=&context.proof;
                        eprintln!("CADENCE_RECEIPT tick={} result_id={} compute_start_ns={} compute_completion_ns={} publication_ns={} retrieval_ns={} actual_receipt_oracle_ns={} checked_compute_ms={:?} checked_completion_to_publication_ms={:?} checked_publication_to_retrieval_ms={:?} checked_retrieval_to_receipt_oracle_ms={:?} publication_to_retrieval_includes_intentional_sleep=true identity=exact_stored_result no_nearest_step_join=true",
                            tick.tick,proof.input.result_id,proof.input.compute_start_unix_ns,proof.compute_completion_unix_ns,context.publication_unix_ns,context.retrieval_unix_ns,receipt.ns,
                            checked_ns(proof.input.compute_start_unix_ns,proof.compute_completion_unix_ns),checked_ns(proof.compute_completion_unix_ns,context.publication_unix_ns),checked_ns(context.publication_unix_ns,context.retrieval_unix_ns),checked_ns(context.retrieval_unix_ns,receipt.ns));
                    }else{eprintln!("CADENCE_RECEIPT tick={} unavailable=missing_exact_proof_context",tick.tick);}
                }else{eprintln!("CADENCE_RECEIPT tick={} unavailable=no_accepted_nonzero_receipt",tick.tick);}
            }
            report_sched("consumer_previous_body_end_to_current_before_sleep",origin,previous.sched_body_end,current.sched_before_sleep);
            // Overlap is timing context only: step_seq is not a result ID.
            for step in probe.control_samples.iter().filter(|step|step.sched_after.end>=previous.start&&step.sched_before.start<=current.start) {
                eprintln!("CADENCE_COMPUTE_OVERLAP outlier_tick={} compute_step_seq={} control_ms={} profile_begin_ms={} profile_end_ms={} probe_lock_wait_ms={} temporal_overlap_only_not_receipt_identity=true",
                    current.tick,step.step_seq,step.control_ms,ms(step.profile_begin),ms(step.profile_end),ms(step.probe_wait));
                report_sched("compute_inner_step_enclosing_read_windows",origin,step.sched_before,step.sched_after);
            }
        }
    }
}
fn checked_ns(a:u64,b:u64)->Option<f64> {
    // command_proof_trace uses zero for an unavailable Unix clock value.
    if a==0||b==0{None}else{b.checked_sub(a).map(|ns|ns as f64/1e6)}
}

#[test]
fn schedstat_parse_is_exact_and_checked() {
    assert_eq!(parse(b"12 34 5\n"),Some(Counters{runtime_ns:12,runqueue_ns:34,timeslices:5}));
    for bad in [&b""[..],b"1 2",b"1 2 3 4",b"-1 2 3",b"18446744073709551616 2 3"] {assert!(parse(bad).is_none());}
}
#[test]
fn scheduler_delta_rejects_missing_and_regressing_samples() {
    let now=Instant::now();
    let a=SchedRead{tid:Some(7),start:now,end:now,counters:Ok(Counters{runtime_ns:10,runqueue_ns:20,timeslices:3})};
    let mut b=a;b.counters=Ok(Counters{runtime_ns:15,runqueue_ns:22,timeslices:4});
    assert_eq!(delta(a,b).unwrap(),Counters{runtime_ns:5,runqueue_ns:2,timeslices:1});
    assert_eq!(delta(a,a).unwrap(),Counters{runtime_ns:0,runqueue_ns:0,timeslices:0});
    b.tid=None;assert!(delta(a,b).is_err());b=a;b.counters=Err("unavailable_open");assert!(delta(a,b).is_err());
    for counters in [Counters{runtime_ns:9,runqueue_ns:20,timeslices:3},
        Counters{runtime_ns:10,runqueue_ns:19,timeslices:3},Counters{runtime_ns:10,runqueue_ns:20,timeslices:2}] {
        b=a;b.counters=Ok(counters);assert!(delta(a,b).is_err());
    }
    b=a;b.tid=Some(8);assert!(delta(a,b).is_err());
    b=a;b.start=now+Duration::from_nanos(1);assert!(delta(a,b).is_err());assert!(delta(b,a).is_err());
    b=a;b.end=now+Duration::from_nanos(1);assert!(delta(b,a).is_err());
}
#[test]
fn cadence_decomposition_uses_previous_body_and_preserves_raw_gap() {
    let t=Instant::now();let end=t+Duration::from_millis(20);let sleep=t+Duration::from_millis(25);let wake=t+Duration::from_millis(90);
    let p=decompose(t,end,sleep,wake).unwrap();
    assert_eq!((p.previous_body,p.intertick,p.wait),(Duration::from_millis(20),Duration::from_millis(5),Duration::from_millis(65)));
    assert_eq!(p.previous_body+p.intertick+p.wait,wake.duration_since(t));
    assert!(decompose(t,sleep,end,wake).is_none());
    for (a,b) in [(9,8),(0,8),(8,0),(0,0)]{assert_eq!(checked_ns(a,b),None);}
    assert_eq!(checked_ns(8,8),Some(0.0));
}
