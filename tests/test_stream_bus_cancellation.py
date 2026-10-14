"""Real synchronous StreamBus interruption and ownership regressions."""
import threading
import unittest

from contracts.streams import StreamFrame
from core.stream_bus import StreamBus


class PublicationInterrupted(BaseException):
    pass


class StreamBusCancellationTests(unittest.TestCase):
    def setUp(self):
        self.bus = StreamBus()
        self.threads, self.releases, self.errors = [], [], []

    def tearDown(self):
        for event in self.releases:
            event.set()
        self.bus.shutdown()
        for thread in self.threads:
            thread.join(3)
            self.assertFalse(thread.is_alive(), 'fixture publisher did not finish')

    def event(self):
        event = threading.Event()
        self.releases.append(event)
        return event

    def start(self, action):
        def run():
            try:
                action()
            except BaseException as exc:
                self.errors.append(exc)
        thread = threading.Thread(target=run)
        self.threads.append(thread)
        thread.start()
        return thread

    @staticmethod
    def frame(value):
        return StreamFrame(topic='test', source='fixture', revision=str(value))

    def test_attempted_item_is_not_replayed_and_next_publication_progresses(self):
        original, seen, tail = PublicationInterrupted('first'), [], []
        def callback(frame):
            seen.append(frame.revision)
            if frame.revision == '1':
                raise original
        self.bus.subscribe(callback)
        self.bus.subscribe(lambda frame: tail.append(frame.revision))
        item = self.bus.prepare_publication(self.frame(1))
        with self.assertRaises(PublicationInterrupted) as caught:
            self.bus.dispatch_prepared(item)
        self.assertIs(caught.exception, original)
        self.assertTrue(item.dispatch_item.completion.is_set())
        self.bus.dispatch_prepared(item)
        self.bus.publish_frame(self.frame(2))
        self.assertEqual(seen, ['1', '2'])
        self.assertEqual(tail, ['2'])
        self.assertFalse(self.bus._dispatch_draining)
        self.assertIsNone(self.bus._dispatch_owner_thread_id)

    def test_concurrent_accepted_publisher_is_released(self):
        entered, release, returned = [self.event() for _ in range(3)]
        original, seen = PublicationInterrupted('owner'), []
        def callback(frame):
            seen.append(frame.revision)
            if frame.revision == '1':
                entered.set()
                self.assertTrue(release.wait(3))
                raise original
        self.bus.subscribe(callback)
        first = self.start(lambda: self.bus.publish_frame(self.frame(1)))
        self.assertTrue(entered.wait(2))
        item = self.bus.prepare_publication(self.frame(2))
        second = self.start(lambda: (self.bus.dispatch_prepared(item), returned.set()))
        self.assertFalse(returned.is_set())
        release.set()
        self.assertTrue(returned.wait(2))
        first.join(2); second.join(2)
        self.assertEqual(self.errors, [original])
        self.assertEqual(seen, ['1', '2'])
        self.assertTrue(item.dispatch_item.completion.is_set())

    def test_reentrant_publication_keeps_fifo_without_duplicate_delivery(self):
        original, seen, tail = PublicationInterrupted('reentrant'), [], []
        def callback(frame):
            seen.append(frame.revision)
            if frame.revision == '1':
                self.bus.publish_frame(self.frame(2))
                raise original
        self.bus.subscribe(callback)
        self.bus.subscribe(lambda frame: tail.append(frame.revision))
        with self.assertRaises(PublicationInterrupted) as caught:
            self.bus.publish_frame(self.frame(1))
        self.assertIs(caught.exception, original)
        self.assertEqual(seen, ['1', '2'])
        self.assertEqual(tail, ['2'])
        self.assertFalse(self.bus._dispatch_queue)

    def test_first_interruption_survives_later_interruptions_and_ordinary_error(self):
        first, second = PublicationInterrupted('first'), PublicationInterrupted('second')
        seen, tail = [], []
        def callback(frame):
            seen.append(frame.revision)
            if frame.revision == '1': raise first
            if frame.revision == '2': raise second
            if frame.revision == '3': raise ValueError('ordinary')
        self.bus.subscribe(callback, topics=('test',))
        self.bus.subscribe(lambda frame: tail.append(frame.revision), topics=('test',))
        items = [self.bus.prepare_publication(self.frame(i)) for i in range(1, 5)]
        with self.assertRaises(PublicationInterrupted) as caught:
            self.bus.dispatch_prepared(items[0])
        self.assertIs(caught.exception, first)
        self.assertEqual(seen, ['1', '2', '3', '4'])
        self.assertEqual(tail, ['3', '4'])
        self.assertTrue(all(item.dispatch_item.completion.is_set() for item in items))
        self.assertIsNotNone(self.bus.latest_frame('diagnostics.stream_bus.subscription_failed'))

    def test_shutdown_releases_queued_waiter_without_delivering_it(self):
        entered, release, returned = [self.event() for _ in range(3)]
        original, seen = PublicationInterrupted('shutdown'), []
        def callback(frame):
            seen.append(frame.revision)
            entered.set()
            self.assertTrue(release.wait(3))
            self.bus.shutdown()
            raise original
        self.bus.subscribe(callback)
        first = self.start(lambda: self.bus.publish_frame(self.frame(1)))
        self.assertTrue(entered.wait(2))
        item = self.bus.prepare_publication(self.frame(2))
        second = self.start(lambda: (self.bus.dispatch_prepared(item), returned.set()))
        release.set()
        self.assertTrue(returned.wait(2))
        first.join(2); second.join(2)
        self.assertEqual(seen, ['1'])
        self.assertEqual(self.errors, [original])
        self.assertTrue(item.dispatch_item.completion.is_set())
        self.assertFalse(self.bus._dispatch_draining)

    def test_old_rethrow_cannot_clear_a_new_drainer_after_empty_handoff(self):
        empty, leave_old, entered, leave_new = [self.event() for _ in range(4)]
        original, seen, old_owner = PublicationInterrupted('handoff'), [], []
        lock, bus = self.bus._lock, self.bus
        class HandoffLock:
            fired = False
            def __enter__(self):
                lock.acquire()
                return self
            def __exit__(self, *exc):
                fire = (not self.fired and old_owner
                        and threading.get_ident() == old_owner[0]
                        and not bus._dispatch_draining and not bus._dispatch_queue)
                if fire:
                    self.fired = True
                lock.release()
                if fire:
                    empty.set()
                    if not leave_old.wait(3):
                        raise AssertionError('old owner was not released')
        bus._lock = HandoffLock()
        def callback(frame):
            seen.append(frame.revision)
            if frame.revision == '1':
                raise original
            if frame.revision == '2':
                entered.set()
                self.assertTrue(leave_new.wait(3))
        bus.subscribe(callback)
        def publish_old():
            old_owner.append(threading.get_ident())
            bus.publish_frame(self.frame(1))
        first = self.start(publish_old)
        self.assertTrue(empty.wait(2))
        second = self.start(lambda: bus.publish_frame(self.frame(2)))
        self.assertTrue(entered.wait(2))
        leave_old.set(); first.join(2)
        self.assertEqual(self.errors, [original])
        with bus._lock:
            self.assertTrue(bus._dispatch_draining)
            self.assertEqual(bus._dispatch_owner_thread_id, second.ident)
        returned = self.event()
        item = bus.prepare_publication(self.frame(3))
        third = self.start(lambda: (bus.dispatch_prepared(item), returned.set()))
        self.assertFalse(returned.is_set())
        leave_new.set()
        self.assertTrue(returned.wait(2))
        second.join(2); third.join(2)
        self.assertEqual(seen, ['1', '2', '3'])
        self.assertFalse(bus._dispatch_draining)
        self.assertIsNone(bus._dispatch_owner_thread_id)
