"""Latch FCU disconnects and boot-clock rollback for one Commander lifetime."""


class FcuGuard:
    def __init__(self, system_id=1, component_id=1):
        if any(type(value) is not int or not 1 <= value <= 255 for value in (system_id,component_id)):
            raise ValueError('invalid FCU MAVLink identity')
        self.system_id = system_id
        self.component_id = component_id
        self.connected_once = False
        self.boot_clocks = {}
        self.fault = ''

    def observe_connection(self, connected):
        if connected:
            self.connected_once = True
        elif self.connected_once:
            self.fault = self.fault or 'FCU connection interrupted; restart Commander for a new mission'

    def observe_boot(self, system_id, component_id, message_id, boot_ms):
        if (system_id, component_id) != (self.system_id, self.component_id):
            return
        if message_id not in (2, 30, 31, 32) or not 0 <= boot_ms <= 0xffffffff:
            return
        previous = self.boot_clocks.get(message_id)
        # Separate clocks avoid cross-stream ordering. Ignore small packet
        # reordering and the normal uint32 rollover after about 49.7 days.
        if previous is not None and boot_ms < previous:
            rollover = previous >= 0xffff0000 and boot_ms < 0x10000
            if previous - boot_ms > 1000 and not rollover:
                self.fault = self.fault or 'FCU boot clock reset; restart Commander for a new mission'
            if not rollover:
                return
        self.boot_clocks[message_id] = boot_ms
