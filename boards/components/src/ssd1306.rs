use capsules::bus;
use capsules::ssd1306::{SSD_Screen, SSD1306};
use capsules::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil::time::{self, Alarm};

#[macro_export]
macro_rules! ssd1306_component_static {
    ($A: ty, $B: ty, $(,)?) => {{
        let buffer = kernel::static_buf!([u8; capsules::ssd1306::BUFFER_SIZE]);
        let command_sequence = kernel::static_buf!(
            [capsules::ssd1306::ScreenCommand; capsules::ssd1306::SEQUENCE_BUFFER_SIZE]
        );
        let ssd1306_alarm =
            kernel::static_buf!(capsules::virtual_alarm::VirtualMuxAlarm<'static, $A>);
        let ssd1306 = kernel::static_buf!(
            capsules::ssd1306::SSD1306<
                'static,
                capsules::virtual_alarm::VirtualMuxAlarm<'static, $A>,
                $B,
            >
        );

        (ssd1306_alarm, ssd1306, command_sequence, buffer)
    };};
}

pub struct SSD1306Component<A: 'static + time::Alarm<'static>, B: 'static + bus::Bus<'static>> {
    alarm_mux: &'static MuxAlarm<'static, A>,
    bus: &'static B,
    screen: &'static SSD_Screen,
}

impl<A: 'static + time::Alarm<'static>, B: 'static + bus::Bus<'static>> SSD1306Component<A, B> {
    pub fn new(
        alarm_mux: &'static MuxAlarm<'static, A>,
        bus: &'static B,
        screen: &'static SSD_Screen,
    ) -> SSD1306Component<A, B> {
        SSD1306Component {
            alarm_mux,
            bus,
            screen,
        }
    }
}

impl<A: 'static + time::Alarm<'static>, B: 'static + bus::Bus<'static>> Component
    for SSD1306Component<A, B>
{
    type StaticInput = (
        &'static mut MaybeUninit<VirtualMuxAlarm<'static, A>>,
        &'static mut MaybeUninit<SSD1306<'static, VirtualMuxAlarm<'static, A>, B>>,
        &'static mut MaybeUninit<
            [capsules::ssd1306::ScreenCommand; capsules::ssd1306::SEQUENCE_BUFFER_SIZE],
        >,
        &'static mut MaybeUninit<[u8; capsules::ssd1306::BUFFER_SIZE]>,
    );

    type Output = &'static SSD1306<'static, VirtualMuxAlarm<'static, A>, B>;

    fn finalize(self, static_memory: Self::StaticInput) -> Self::Output {
        let ssd1306_alarm = static_memory.0.write(VirtualMuxAlarm::new(self.alarm_mux));
        ssd1306_alarm.setup();

        let command_sequence = static_memory.2.write(
            [capsules::ssd1306::ScreenCommand {
                id: capsules::ssd1306::CommandId::Nop,
                parameters: None,
            }; capsules::ssd1306::SEQUENCE_BUFFER_SIZE],
        );
        let command_arguments = static_memory.3.write([0; capsules::ssd1306::BUFFER_SIZE]);

        let ssd1306 = static_memory.1.write(SSD1306::new(
            self.bus,
            ssd1306_alarm,
            self.screen,
            command_sequence,
            command_arguments,
        ));
        self.bus.set_client(ssd1306);
        ssd1306_alarm.set_alarm_client(ssd1306);

        ssd1306
    }
}
