// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2024.
// Copyright OxidOS Automotive 2026.

#![no_std]
#![no_main]

use capsules_extra::test::sha256_hw::TestSha256;
use kernel::capabilities;
use kernel::component::Component;
use kernel::debug::PanicResources;
use kernel::deferred_call::DeferredCallClient;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::utilities::single_thread_value::SingleThreadValue;
use kernel::{create_capability, static_init};

use stm32u545::gpio::PinId;
use stm32u545::hash::Hash;

pub mod io;

extern "C" {
    static _sappmem: u8;
    static _eappmem: u8;
}

const NUM_PROCS: usize = 4;

type ChipHw =
    stm32u545::chip::Stm32u5xx<'static, stm32u545::chip::Stm32u5xxDefaultPeripherals<'static>>;
type ProcessPrinterInUse = capsules_system::process_printer::ProcessPrinterText;

static PANIC_RESOURCES: SingleThreadValue<PanicResources<ChipHw, ProcessPrinterInUse>> =
    SingleThreadValue::new(PanicResources::new());

struct NucleoU545RE {
    console: &'static capsules_core::console::Console<'static>,
    scheduler: &'static components::sched::round_robin::RoundRobinComponentType,
    systick: cortexm33::systick::SysTick,
    led: &'static capsules_core::led::LedDriver<
        'static,
        kernel::hil::led::LedHigh<'static, stm32u545::gpio::Pin<'static>>,
        1,
    >,
    button: &'static capsules_core::button::Button<'static, stm32u545::gpio::Pin<'static>>,
    alarm: &'static capsules_core::alarm::AlarmDriver<
        'static,
        capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<
            'static,
            stm32u545::tim::Tim2<'static>,
        >,
    >,
    hash: &'static capsules_extra::test::sha256_hw::TestSha256<
        'static,
        stm32u545::hash::Hash<'static>,
    >,
}

impl SyscallDriverLookup for NucleoU545RE {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules_core::console::DRIVER_NUM => f(Some(self.console)),
            capsules_core::led::DRIVER_NUM => f(Some(self.led)),
            capsules_core::button::DRIVER_NUM => f(Some(self.button)),
            capsules_core::alarm::DRIVER_NUM => f(Some(self.alarm)),
            _ => f(None),
        }
    }
}

impl KernelResources<ChipHw> for NucleoU545RE {
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = components::sched::round_robin::RoundRobinComponentType;
    type SchedulerTimer = cortexm33::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

/// Helper function for board-specific pin muxing
unsafe fn set_pin_primary_functions(periphs: &stm32u545::Stm32u5xxPeripherals) {
    use kernel::hil::gpio::Configure;

    // USART1 Pins (PA9/10)
    let pin9 = periphs.gpio_a.pin(PinId::Pin09);
    let pin10 = periphs.gpio_a.pin(PinId::Pin10);
    pin9.set_mode(stm32u545::gpio::Mode::AlternateFunction);
    pin9.set_alternate_function(7);
    pin9.set_speed_high();
    pin10.set_mode(stm32u545::gpio::Mode::AlternateFunction);
    pin10.set_alternate_function(7);
    pin10.set_speed_high();

    // LED Pin (PA5)
    periphs.gpio_a.pin(PinId::Pin05).make_output();

    // Button Pin (PC13) - Hardware is Active High
    let btn = periphs.gpio_c.pin(PinId::Pin13);
    btn.make_input();
    btn.set_floating_state(kernel::hil::gpio::FloatingState::PullDown);
}

#[inline(never)]
#[allow(clippy::large_stack_arrays)]
unsafe fn start() -> (
    &'static kernel::Kernel,
    &'static NucleoU545RE,
    &'static ChipHw,
) {
    stm32u545::init();

    kernel::deferred_call::initialize_deferred_call_state::<
        <ChipHw as kernel::platform::chip::Chip>::ThreadIdProvider,
    >();

    // Create Individual Drivers
    let exti = stm32u545::exti::init();
    let dma1 = stm32u545::dma::init();
    let usart1 = stm32u545::usart::init();
    let hash = stm32u545::hash::init();

    hash.register();

    // Link DMA to USART1
    usart1.set_dma(dma1, 0, 1);

    // Load Peripherals Bundle
    let periphs = static_init!(
        stm32u545::Stm32u5xxPeripherals<'static>,
        stm32u545::Stm32u5xxPeripherals::new(exti, dma1, usart1, hash)
    );

    // Power and Wires
    periphs.rcc.enable_dma1();
    periphs.rcc.enable_gpioa();
    periphs.rcc.enable_gpioc();
    periphs.rcc.enable_usart1();
    periphs.rcc.enable_hash();
    periphs.rcc.enable_syscfg();
    periphs.rcc.set_usart1_source_pclk();
    periphs.tim2.start();

    set_pin_primary_functions(periphs);

    // Driver Config
    use kernel::hil::uart::Configure;
    let _ = periphs.usart1.configure(kernel::hil::uart::Parameters {
        baud_rate: 115200,
        stop_bits: kernel::hil::uart::StopBits::One,
        parity: kernel::hil::uart::Parity::None,
        hw_flow_control: false,
        width: kernel::hil::uart::Width::Eight,
    });

    // Kernel and Muxes
    let processes = components::process_array::ProcessArrayComponent::new()
        .finalize(components::process_array_component_static!(NUM_PROCS));
    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(processes.as_slice()));

    let uart_mux = components::console::UartMuxComponent::new(periphs.usart1, 115200)
        .finalize(components::uart_mux_component_static!());

    let alarm_mux = components::alarm::AlarmMuxComponent::new(&periphs.tim2).finalize(
        components::alarm_mux_component_static!(stm32u545::tim::Tim2),
    );

    // Capsules
    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules_core::console::DRIVER_NUM,
        uart_mux,
    )
    .finalize(components::console_component_static!());

    components::debug_writer::DebugWriterComponent::new::<
        <ChipHw as kernel::platform::chip::Chip>::ThreadIdProvider,
    >(
        uart_mux,
        create_capability!(capabilities::SetDebugWriterCapability),
    )
    .finalize(components::debug_writer_component_static!());

    let process_console = components::process_console::ProcessConsoleComponent::new(
        board_kernel,
        uart_mux,
        alarm_mux,
        components::process_printer::ProcessPrinterTextComponent::new()
            .finalize(components::process_printer_text_component_static!()),
        None,
    )
    .finalize(components::process_console_component_static!(
        stm32u545::tim::Tim2
    ));
    let _ = process_console.start();

    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules_core::alarm::DRIVER_NUM,
        alarm_mux,
    )
    .finalize(components::alarm_component_static!(stm32u545::tim::Tim2));

    let led_pin = static_init!(stm32u545::gpio::Pin, periphs.gpio_a.pin(PinId::Pin05));
    let led = components::led::LedsComponent::new().finalize(components::led_component_static!(
        kernel::hil::led::LedHigh<'static, stm32u545::gpio::Pin>,
        kernel::hil::led::LedHigh::new(led_pin)
    ));

    let button = components::button::ButtonComponent::new(
        board_kernel,
        capsules_core::button::DRIVER_NUM,
        components::button_component_helper!(
            stm32u545::gpio::Pin,
            (
                static_init!(stm32u545::gpio::Pin, periphs.gpio_c.pin(PinId::Pin13)),
                kernel::hil::gpio::ActivationMode::ActiveHigh,
                kernel::hil::gpio::FloatingState::PullDown
            )
        ),
    )
    .finalize(components::button_component_static!(stm32u545::gpio::Pin));
    // let hash_data_buffer = static_init!([u8; 6], [0x61, 0x62, 0x63, 0x64, 0x65, 0x66]);

    // let hash_digest_buffer = static_init!(
    //     [u8; 32],
    //     [
    //         0xbe, 0xf5, 0x7e, 0xc7, 0xf5, 0x3a, 0x6d, 0x40, 0xbe, 0xb6, 0x40, 0xa7, 0x80, 0xa6,
    //         0x39, 0xc8, 0x3b, 0xc2, 0x9a, 0xc8, 0xa9, 0x81, 0x6f, 0x1f, 0xc6, 0xc5, 0xc6, 0xdc,
    //         0xd9, 0x3c, 0x47, 0x21
    //     ]
    // );

    let hash_data_buffer = static_init!(
        [u8; 108],
        [
            0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x52, 0x75, 0x73, 0x74, 0x79, 0x21, 0x20,
            0x57, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x70, 0x6f, 0x72, 0x74, 0x69, 0x6e, 0x67,
            0x20, 0x48, 0x41, 0x53, 0x48, 0x20, 0x70, 0x65, 0x72, 0x69, 0x70, 0x68, 0x65, 0x72,
            0x61, 0x6c, 0x20, 0x74, 0x6f, 0x20, 0x54, 0x6f, 0x63, 0x6b, 0x20, 0x4f, 0x53, 0x2e,
            0x20, 0x4e, 0x6f, 0x77, 0x20, 0x77, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x74, 0x72,
            0x79, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20, 0x63, 0x68, 0x65, 0x63, 0x6b, 0x20,
            0x69, 0x6e, 0x74, 0x65, 0x72, 0x72, 0x75, 0x70, 0x74, 0x73, 0x20, 0x61, 0x6e, 0x64,
            0x20, 0x62, 0x75, 0x66, 0x66, 0x65, 0x72, 0x69, 0x6e, 0x67
        ]
    );

    let hash_digest_buffer = static_init!(
        [u8; 32],
        [
            0x3d, 0x85, 0xf3, 0x1b, 0x87, 0xb9, 0x9b, 0x20, 0x22, 0x0a, 0x00, 0x7a, 0x27, 0x4f,
            0x30, 0xd6, 0x54, 0xf5, 0xc9, 0xec, 0x30, 0x75, 0x4f, 0xe1, 0x3c, 0x02, 0xd2, 0x0d,
            0x14, 0xa1, 0xba, 0xe7
        ]
    );

    // let hash_data_buffer = static_init!(
    //     [u8; 70],
    //     [
    //         0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x52, 0x75, 0x73, 0x74, 0x79, 0x21, 0x20,
    //         0x57, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x70, 0x6f, 0x72, 0x74, 0x69, 0x6e, 0x67,
    //         0x20, 0x48, 0x41, 0x53, 0x48, 0x20, 0x70, 0x65, 0x72, 0x69, 0x70, 0x68, 0x65, 0x72,
    //         0x61, 0x6c, 0x20, 0x74, 0x6f, 0x20, 0x54, 0x6f, 0x63, 0x6b, 0x20, 0x4f, 0x53, 0x2e,
    //         0x20, 0x4e, 0x6f, 0x77, 0x20, 0x77, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x74, 0x72,
    //     ]
    // );

    // let hash_digest_buffer = static_init!(
    //     [u8; 32],
    //     [
    //         0x4e, 0xd3, 0xe3, 0xe2, 0x3c, 0x13, 0x3c, 0xff, 0xf3, 0xe8, 0x86, 0x67, 0x06, 0xa6,
    //         0x59, 0x0c, 0xdc, 0xa4, 0x6d, 0xd7, 0xff, 0x0b, 0x51, 0x88, 0x55, 0x03, 0x67, 0x7a,
    //         0x83, 0x9c, 0x19, 0x94
    //     ]
    // );

    let test_hash = static_init!(
        TestSha256<'static, Hash<'static>>,
        TestSha256::new(hash, hash_data_buffer, hash_digest_buffer, true)
    );

    // Platform and Interrupts
    let platform = static_init!(
        NucleoU545RE,
        NucleoU545RE {
            console,
            scheduler: components::sched::round_robin::RoundRobinComponent::new(processes)
                .finalize(components::round_robin_component_static!(NUM_PROCS)),
            systick: cortexm33::systick::SysTick::new(),
            led,
            button,
            alarm,
            hash: test_hash
        }
    );

    // Initialize Chip
    let default_peripherals = static_init!(
        stm32u545::chip::Stm32u5xxDefaultPeripherals,
        stm32u545::chip::Stm32u5xxDefaultPeripherals::new(
            &periphs.tim2,
            periphs.usart1,
            periphs.exti,
            periphs.hash
        )
    );

    let chip = static_init!(
        stm32u545::chip::Stm32u5xx<stm32u545::chip::Stm32u5xxDefaultPeripherals>,
        stm32u545::chip::Stm32u5xx::new(default_peripherals)
    );

    // --- LOAD PROCESSES ---
    let app_flash = core::slice::from_raw_parts(
        core::ptr::from_ref(&_sappmem),
        core::ptr::from_ref(&_eappmem) as usize - core::ptr::from_ref(&_sappmem) as usize,
    );
    let app_memory = static_init!([u8; 98304], [0; 98304]);

    let _ = kernel::process::load_processes(
        board_kernel,
        chip,
        app_flash,
        app_memory,
        &capsules_system::process_policies::PanicFaultPolicy {},
        &create_capability!(capabilities::ProcessManagementCapability),
    );

    (board_kernel, platform, chip)
}

#[no_mangle]
pub unsafe fn main() {
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);

    let (board_kernel, platform, chip) = start();

    platform.hash.run();

    // Hand over control to the Tock Kernel Loop
    board_kernel.kernel_loop::<NucleoU545RE, ChipHw, { NUM_PROCS as u8 }>(
        platform,
        chip,
        None,
        &main_loop_capability,
    );
}
