// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2024.
// Copyright OxidOS Automotive 2026.

#![no_std]
#![no_main]

use capsules_extra::test::hmac_md5::TestHmacMd5;
use capsules_extra::test::hmac_sha1::TestHmacSha1;
use capsules_extra::test::hmac_sha224::TestHmacSha224;
use capsules_extra::test::hmac_sha256::TestHmacSha256;
use capsules_extra::test::md5::TestMd5;
use capsules_extra::test::sha1::TestSha1;
use capsules_extra::test::sha224::TestSha224;
use capsules_extra::test::sha256::TestSha256;
use kernel::capabilities;
use kernel::component::Component;
use kernel::debug::PanicResources;
use kernel::deferred_call::DeferredCallClient;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::utilities::single_thread_value::SingleThreadValue;
use kernel::{create_capability, static_init};

use stm32u545::gpio::PinId;

use stm32u545::hash::md5::Md5Adapter;
use stm32u545::hash::sha1::Sha1Adapter;
use stm32u545::hash::sha224::{self, Sha224Adapter};
use stm32u545::hash::traits::HashAdaptee;

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
    SingleThreadValue::new();

kernel::stack_size! {0x2000}

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
    // SHA256
    // hash: &'static capsules_extra::test::sha256::TestSha256<
    //     'static,
    //     stm32u545::hash::sha256::Sha256Adapter<'static>,
    // >,
    // HMAC-SHA256
    // hash: &'static capsules_extra::test::hmac_sha256::TestHmacSha256<
    //     'static,
    //     stm32u545::hash::md5::Md5Adapter<'static>,
    // >,
    // MD5
    // hash: &'static capsules_extra::test::md5::TestMd5<
    //     'static,
    //     stm32u545::hash::md5::Md5Adapter<'static>,
    // >,
    // HMAC-MD5
    // hash: &'static capsules_extra::test::hmac_md5::TestHmacMd5<
    //     'static,
    //     stm32u545::hash::md5::Md5Adapter<'static>,
    // >,
    // SHA224
    // hash: &'static capsules_extra::test::sha224::TestSha224<
    //     'static,
    //     stm32u545::hash::sha224::Sha224Adapter<'static>,
    // >,
    // HMAC-SHA224
    hash: &'static capsules_extra::test::hmac_sha224::TestHmacSha224<
        'static,
        stm32u545::hash::sha224::Sha224Adapter<'static>,
    >,
    // SHA1
    // hash: &'static capsules_extra::test::sha1::TestSha1<
    //     'static,
    //     stm32u545::hash::sha1::Sha1Adapter<'static>,
    // >,
    // HMAC-SHA1
    // hash: &'static capsules_extra::test::hmac_sha1::TestHmacSha1<
    //     'static,
    //     stm32u545::hash::sha1::Sha1Adapter<'static>,
    // >,
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
unsafe fn set_pin_primary_functions(periphs: &stm32u545::chip::Stm32u5xxDefaultPeripherals) {
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
    let exti = static_init!(
        stm32u545::exti::Exti<'static>,
        stm32u545::exti::Exti::new(stm32u545::exti::EXTI_BASE)
    );
    let dma1 = static_init!(
        stm32u545::dma::Dma,
        stm32u545::dma::Dma::new(stm32u545::dma::DMA1_BASE)
    );
    let usart1 = static_init!(
        stm32u545::usart::Usart<'static>,
        stm32u545::usart::Usart::new(stm32u545::usart::USART1_BASE)
    );

    let hash = static_init!(
        stm32u545::hash::Hash<'static>,
        stm32u545::hash::Hash::new(stm32u545::hash::HASH_BASE)
    );

    // let hash = stm32u545::hash::init();
    hash.register();

    // Load Peripherals Bundle
    let periphs = static_init!(
        stm32u545::chip::Stm32u5xxDefaultPeripherals<'static>,
        stm32u545::chip::Stm32u5xxDefaultPeripherals::new(usart1, exti, dma1, hash)
    );

    // Initialize wiring (DMA, clocks)
    periphs.init();

    // Board specific wiring
    periphs.tim2.start();
    set_pin_primary_functions(periphs);

    // let sha256 = static_init!(
    //     stm32u545::hash::sha256::Sha256Adapter<'static>,
    //     stm32u545::hash::sha256::Sha256Adapter::new(hash)
    // );
    // let md5 = static_init!(
    //     stm32u545::hash::md5::Md5Adapter<'static>,
    //     stm32u545::hash::md5::Md5Adapter::new(hash)
    // );

    let sha224 = static_init!(
        stm32u545::hash::sha224::Sha224Adapter<'static>,
        stm32u545::hash::sha224::Sha224Adapter::new(hash)
    );

    // let sha1 = static_init!(
    //     stm32u545::hash::sha1::Sha1Adapter<'static>,
    //     stm32u545::hash::sha1::Sha1Adapter::new(hash)
    // );

    hash.set_adapter(sha224);

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

    // Various test cases
    // Test 1: Simple SHA256
    // let hash_data_buffer = static_init!([u8; 6], [0x61, 0x62, 0x63, 0x64, 0x65, 0x66]);

    // let hash_digest_buffer = static_init!(
    //     [u8; 32],
    //     [
    //         0xbe, 0xf5, 0x7e, 0xc7, 0xf5, 0x3a, 0x6d, 0x40, 0xbe, 0xb6, 0x40, 0xa7, 0x80, 0xa6,
    //         0x39, 0xc8, 0x3b, 0xc2, 0x9a, 0xc8, 0xa9, 0x81, 0x6f, 0x1f, 0xc6, 0xc5, 0xc6, 0xdc,
    //         0xd9, 0x3c, 0x47, 0x21
    //     ]
    // );

    // Test 2: Less than FIFO size
    // let hash_data_buffer = static_init!(
    //     [u8; 56],
    //     [
    //         0x61, 0x62, 0x63, 0x64, 0x62, 0x63, 0x64, 0x65, 0x63, 0x64, 0x65, 0x66, 0x64, 0x65,
    //         0x66, 0x67, 0x65, 0x66, 0x67, 0x68, 0x66, 0x67, 0x68, 0x69, 0x67, 0x68, 0x69, 0x6a,
    //         0x68, 0x69, 0x6a, 0x6b, 0x69, 0x6a, 0x6b, 0x6c, 0x6a, 0x6b, 0x6c, 0x6d, 0x6b, 0x6c,
    //         0x6d, 0x6e, 0x6c, 0x6d, 0x6e, 0x6f, 0x6d, 0x6e, 0x6f, 0x70, 0x6e, 0x6f, 0x70, 0x71
    //     ]
    // );

    // let hash_digest_buffer = static_init!(
    //     [u8; 32],
    //     [
    //         0x24, 0x8d, 0x6a, 0x61, 0xd2, 0x06, 0x38, 0xb8, 0xe5, 0xc0, 0x26, 0x93, 0x0c, 0x3e,
    //         0x60, 0x39, 0xa3, 0x3c, 0xe4, 0x59, 0x64, 0xff, 0x21, 0x67, 0xf6, 0xec, 0xed, 0xd4,
    //         0x19, 0xdb, 0x06, 0xc1
    //     ]
    // );

    // Test 3: SHA256 with big message
    // let hash_data_buffer = static_init!(
    //     [u8; 108],
    //     [
    //         0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x52, 0x75, 0x73, 0x74, 0x79, 0x21, 0x20,
    //         0x57, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x70, 0x6f, 0x72, 0x74, 0x69, 0x6e, 0x67,
    //         0x20, 0x48, 0x41, 0x53, 0x48, 0x20, 0x70, 0x65, 0x72, 0x69, 0x70, 0x68, 0x65, 0x72,
    //         0x61, 0x6c, 0x20, 0x74, 0x6f, 0x20, 0x54, 0x6f, 0x63, 0x6b, 0x20, 0x4f, 0x53, 0x2e,
    //         0x20, 0x4e, 0x6f, 0x77, 0x20, 0x77, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x74, 0x72,
    //         0x79, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20, 0x63, 0x68, 0x65, 0x63, 0x6b, 0x20,
    //         0x69, 0x6e, 0x74, 0x65, 0x72, 0x72, 0x75, 0x70, 0x74, 0x73, 0x20, 0x61, 0x6e, 0x64,
    //         0x20, 0x62, 0x75, 0x66, 0x66, 0x65, 0x72, 0x69, 0x6e, 0x67
    //     ]
    // );

    // let hash_digest_buffer = static_init!(
    //     [u8; 32],
    //     [
    //         0x3d, 0x85, 0xf3, 0x1b, 0x87, 0xb9, 0x9b, 0x20, 0x22, 0x0a, 0x00, 0x7a, 0x27, 0x4f,
    //         0x30, 0xd6, 0x54, 0xf5, 0xc9, 0xec, 0x30, 0x75, 0x4f, 0xe1, 0x3c, 0x02, 0xd2, 0x0d,
    //         0x14, 0xa1, 0xba, 0xe7
    //     ]
    // );

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
    //
    //

    // HMAC
    // let hmac_key = static_init!([u8; 1], [0x12]);

    // let hash_data_buffer = static_init!([u8; 4], [0x61, 0x62, 0x63, 0x64]);

    // let hash_digest_buffer = static_init!([u8; 32], [0u8; 32]);
    // let correct = static_init!(
    //     [u8; 32],
    //     [
    //         0x9e, 0xc9, 0x2b, 0xb2, 0xe6, 0xdf, 0x61, 0x8f, 0x3e, 0x4d, 0x4a, 0x31, 0xf5, 0xe5,
    //         0x27, 0xc2, 0x49, 0x6e, 0xae, 0x9b, 0x09, 0xc1, 0xe0, 0xcd, 0xe1, 0x33, 0x9c, 0xe2,
    //         0x65, 0x00, 0xc6, 0x76
    //     ]
    // );
    //
    // Working example
    // let hmac_key = static_init!([u8; 1], [0x12]);

    // let hash_data_buffer = static_init!([u8; 1], [0x61]);

    // let hash_digest_buffer = static_init!([u8; 32], [0u8; 32]);
    // let correct = static_init!(
    //     [u8; 32],
    //     [
    //         0xed, 0x71, 0xe8, 0xc5, 0xc8, 0x2a, 0x5c, 0x8b, 0x25, 0xa8, 0xe7, 0xc1, 0xd2, 0xec,
    //         0x04, 0x18, 0xae, 0x80, 0x0b, 0xfb, 0x99, 0xbb, 0x75, 0x47, 0x7c, 0x23, 0x6c, 0x65,
    //         0x1a, 0x44, 0x80, 0xea
    //     ]
    // );

    // Trying bigger key and data
    // DMA interrupt doesn't fire
    // let hmac_key = static_init!([u8; 6], [0x12, 0x34, 0x56, 0x78, 0x91, 0x01]);

    // let hash_data_buffer = static_init!([u8; 5], [0x61, 0x62, 0x63, 0x64, 0x65]);

    // let hash_digest_buffer = static_init!([u8; 32], [0u8; 32]);
    // let correct = static_init!(
    //     [u8; 32],
    //     [
    //         0x3a, 0x4b, 0x96, 0x9a, 0x96, 0xa9, 0xc9, 0x52, 0x0e, 0xcd, 0xdd, 0x4d, 0xef, 0x05,
    //         0x64, 0xfb, 0x1c, 0x79, 0xe6, 0xc3, 0x30, 0xa6, 0x61, 0x8f, 0x3b, 0xdc, 0xbb, 0x0f,
    //         0xd6, 0xaf, 0x55, 0x44
    //     ]
    // );

    // let hash_data_buffer = static_init!(
    //     [u8; 113],
    //     [
    //         0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x52, 0x75, 0x73, 0x74, 0x79, 0x21, 0x20,
    //         0x57, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x70, 0x6f, 0x72, 0x74, 0x69, 0x6e, 0x67,
    //         0x20, 0x48, 0x41, 0x53, 0x48, 0x20, 0x70, 0x65, 0x72, 0x69, 0x70, 0x68, 0x65, 0x72,
    //         0x61, 0x6c, 0x20, 0x74, 0x6f, 0x20, 0x54, 0x6f, 0x63, 0x6b, 0x20, 0x4f, 0x53, 0x2e,
    //         0x20, 0x4e, 0x6f, 0x77, 0x20, 0x77, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x74, 0x72,
    //         0x79, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20, 0x63, 0x68, 0x65, 0x63, 0x6b, 0x20,
    //         0x69, 0x6e, 0x74, 0x65, 0x72, 0x72, 0x75, 0x70, 0x74, 0x73, 0x20, 0x61, 0x6e, 0x64,
    //         0x20, 0x62, 0x75, 0x66, 0x66, 0x65, 0x72, 0x69, 0x6e, 0x67, 0x96, 0x12, 0xff, 0x45,
    //         0x11
    //     ]
    // );

    // let hash_digest_buffer = static_init!([u8; 32], [0u8; 32]);
    // let correct = static_init!(
    //     [u8; 32],
    //     [
    //         0xbb, 0x98, 0x4f, 0xc1, 0xd0, 0xf8, 0x38, 0x25, 0x52, 0xcc, 0x85, 0xda, 0xb7, 0x24,
    //         0x81, 0x11, 0xca, 0xa1, 0xcb, 0x82, 0x69, 0x75, 0xc5, 0x41, 0x1a, 0x33, 0x48, 0x96,
    //         0xb3, 0xa9, 0xb3, 0xf0
    //     ]
    // );
    // let hmac_key = static_init!(
    //     [u8; 25],
    //     [
    //         0x12, 0x34, 0x56, 0x78, 0x98, 0x76, 0x54, 0x32, 0x12, 0x34, 0x56, 0x76, 0x54, 0x32,
    //         0x12, 0x34, 0x56, 0x76, 0x54, 0x32, 0x23, 0x45, 0x67, 0x65, 0x43
    //     ]
    // );

    // VERY BIG EXAMPLE
    //

    // let hash_data_buffer = static_init!(
    //     [u8; 113],
    //     [
    //         0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x52, 0x75, 0x73, 0x74, 0x79, 0x21, 0x20,
    //         0x57, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x70, 0x6f, 0x72, 0x74, 0x69, 0x6e, 0x67,
    //         0x20, 0x48, 0x41, 0x53, 0x48, 0x20, 0x70, 0x65, 0x72, 0x69, 0x70, 0x68, 0x65, 0x72,
    //         0x61, 0x6c, 0x20, 0x74, 0x6f, 0x20, 0x54, 0x6f, 0x63, 0x6b, 0x20, 0x4f, 0x53, 0x2e,
    //         0x20, 0x4e, 0x6f, 0x77, 0x20, 0x77, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x74, 0x72,
    //         0x79, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20, 0x63, 0x68, 0x65, 0x63, 0x6b, 0x20,
    //         0x69, 0x6e, 0x74, 0x65, 0x72, 0x72, 0x75, 0x70, 0x74, 0x73, 0x20, 0x61, 0x6e, 0x64,
    //         0x20, 0x62, 0x75, 0x66, 0x66, 0x65, 0x72, 0x69, 0x6e, 0x67, 0x96, 0x12, 0xff, 0x45,
    //         0x11
    //     ]
    // );

    // let hash_digest_buffer = static_init!([u8; 32], [0u8; 32]);
    // let correct = static_init!(
    //     [u8; 32],
    //     [
    //         0xcc, 0xff, 0xe0, 0x1c, 0x3c, 0x4b, 0xd7, 0x47, 0x68, 0xc2, 0xb4, 0xa8, 0xb3, 0x48,
    //         0xf4, 0x63, 0x59, 0x6f, 0x27, 0x9a, 0x09, 0xe9, 0xfb, 0x45, 0x57, 0xde, 0x4a, 0xcd,
    //         0x20, 0x9e, 0x21, 0xe5
    //     ]
    // );
    // let hmac_key = static_init!(
    //     [u8; 108],
    //     [
    //         0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x52, 0x75, 0x73, 0x74, 0x79, 0x21, 0x20,
    //         0x57, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x70, 0x6f, 0x72, 0x74, 0x69, 0x6e, 0x67,
    //         0x20, 0x48, 0x41, 0x53, 0x48, 0x20, 0x70, 0x65, 0x72, 0x69, 0x70, 0x68, 0x65, 0x72,
    //         0x61, 0x6c, 0x20, 0x74, 0x6f, 0x20, 0x54, 0x6f, 0x63, 0x6b, 0x20, 0x4f, 0x53, 0x2e,
    //         0x20, 0x4e, 0x6f, 0x77, 0x20, 0x77, 0x65, 0x20, 0x61, 0x72, 0x65, 0x20, 0x74, 0x72,
    //         0x79, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20, 0x63, 0x68, 0x65, 0x63, 0x6b, 0x20,
    //         0x69, 0x6e, 0x74, 0x65, 0x72, 0x72, 0x75, 0x70, 0x74, 0x73, 0x20, 0x61, 0x6e, 0x64,
    //         0x20, 0x62, 0x75, 0x66, 0x66, 0x65, 0x72, 0x69, 0x6e, 0x67
    //     ]
    // );

    // let test_hash = static_init!(
    //     TestHmacSha256<'static, stm32u545::hash::sha256::Sha256Adapter<'static>>,
    //     TestHmacSha256::new(
    //         sha256,
    //         hmac_key,
    //         hash_data_buffer,
    //         hash_digest_buffer,
    //         correct
    //     )
    // );

    // let test_hash = static_init!(
    //     TestSha256<'static, Sha256Adapter<'static>>,
    //     TestSha256::new(sha256, hash_data_buffer, hash_digest_buffer, true)
    // );
    //

    // MD5
    // let hash_data_buffer = static_init!([u8; 4], [0x61, 0x62, 0x63, 0x64]);

    // let hash_digest_buffer = static_init!(
    //     [u8; 16],
    //     [
    //         0xe2, 0xfc, 0x71, 0x4c, 0x47, 0x27, 0xee, 0x93, 0x95, 0xf3, 0x24, 0xcd, 0x2e, 0x7f,
    //         0x33, 0x1f,
    //     ]
    // );

    // let test_hash = static_init!(
    //     TestMd5<'static, Md5Adapter<'static>>,
    //     TestMd5::new(md5, hash_data_buffer, hash_digest_buffer, true)
    // );
    // HMAC-MD5

    // let hmac_key = static_init!([u8; 6], [0x12, 0x34, 0x56, 0x78, 0x91, 0x01]);

    // let hash_data_buffer = static_init!([u8; 5], [0x61, 0x62, 0x63, 0x64, 0x65]);

    // let hash_digest_buffer = static_init!([u8; 16], [0u8; 16]);
    // let correct = static_init!(
    //     [u8; 16],
    //     [
    //         0xc8, 0x27, 0x73, 0xee, 0xb9, 0x9c, 0x7b, 0xf0, 0x12, 0xc1, 0xbc, 0xb6, 0x5b, 0x87,
    //         0xa1, 0x12
    //     ]
    // );
    // let test_hash = static_init!(
    //     TestHmacMd5<'static, Md5Adapter<'static>>,
    //     TestHmacMd5::new(md5, hmac_key, hash_data_buffer, hash_digest_buffer, correct)
    // );

    // SHA1
    // let hash_data_buffer = static_init!([u8; 4], [0x61, 0x62, 0x63, 0x64]);

    // let hash_digest_buffer = static_init!(
    //     [u8; 20],
    //     [
    //         0x81, 0xfe, 0x8b, 0xfe, 0x87, 0x57, 0x6c, 0x3e, 0xcb, 0x22, 0x42, 0x6f, 0x8e, 0x57,
    //         0x84, 0x73, 0x82, 0x91, 0x7a, 0xcf
    //     ]
    // );
    // let test_hash = static_init!(
    //     TestSha1<'static, Sha1Adapter<'static>>,
    //     TestSha1::new(sha1, hash_data_buffer, hash_digest_buffer, true)
    // );

    // HMAC-SHA1
    // let hmac_key = static_init!([u8; 6], [0x12, 0x34, 0x56, 0x78, 0x91, 0x01]);

    // let hash_data_buffer = static_init!([u8; 5], [0x61, 0x62, 0x63, 0x64, 0x65]);

    // let hash_digest_buffer = static_init!([u8; 20], [0u8; 20]);
    // let correct = static_init!(
    //     [u8; 20],
    //     [
    //         0x1a, 0x44, 0xb6, 0xc8, 0xf1, 0xa5, 0x60, 0x98, 0x5b, 0xb8, 0x72, 0x1e, 0x8f, 0x70,
    //         0x42, 0xc3, 0x08, 0x21, 0xf5, 0xb2
    //     ]
    // );
    // let test_hash = static_init!(
    //     TestHmacSha1<'static, Sha1Adapter<'static>>,
    //     TestHmacSha1::new(
    //         sha1,
    //         hmac_key,
    //         hash_data_buffer,
    //         hash_digest_buffer,
    //         correct
    //     )
    // );
    //
    // SHA224
    // let hash_data_buffer = static_init!([u8; 4], [0x61, 0x62, 0x63, 0x64]);

    // let hash_digest_buffer = static_init!(
    //     [u8; 28],
    //     [
    //         0xa7, 0x66, 0x54, 0xd8, 0xe3, 0x55, 0x0e, 0x9a, 0x2d, 0x67, 0xa0, 0xee, 0xb6, 0xc6,
    //         0x7b, 0x22, 0x0e, 0x58, 0x85, 0xed, 0xdd, 0x3f, 0xde, 0x13, 0x58, 0x06, 0xe6, 0x01
    //     ]
    // );

    // let test_hash = static_init!(
    //     TestSha224<'static, Sha224Adapter<'static>>,
    //     TestSha224::new(sha224, hash_data_buffer, hash_digest_buffer, true)
    // );
    // HMAC-SHA224
    let hmac_key = static_init!([u8; 6], [0x12, 0x34, 0x56, 0x78, 0x91, 0x01]);

    let hash_data_buffer = static_init!([u8; 5], [0x61, 0x62, 0x63, 0x64, 0x65]);

    let hash_digest_buffer = static_init!([u8; 28], [0u8; 28]);
    let correct = static_init!(
        [u8; 28],
        [
            0x50, 0x64, 0x2b, 0x2c, 0xfe, 0x92, 0xa5, 0x2e, 0xd5, 0x99, 0x81, 0x97, 0xcc, 0x14,
            0x2b, 0x44, 0x71, 0xea, 0xd6, 0xcc, 0x71, 0x97, 0xc8, 0x67, 0xc2, 0x64, 0x9a, 0x86
        ]
    );
    let test_hash = static_init!(
        TestHmacSha224<'static, Sha224Adapter<'static>>,
        TestHmacSha224::new(
            sha224,
            hmac_key,
            hash_data_buffer,
            hash_digest_buffer,
            correct
        )
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

    let chip = static_init!(
        stm32u545::chip::Stm32u5xx<stm32u545::chip::Stm32u5xxDefaultPeripherals>,
        stm32u545::chip::Stm32u5xx::new(periphs)
    );

    // Symbols for linker
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    // Load processes
    let app_flash = core::slice::from_raw_parts(
        core::ptr::addr_of!(_sapps),
        core::ptr::addr_of!(_eapps) as usize - core::ptr::addr_of!(_sapps) as usize,
    );

    let app_memory = core::slice::from_raw_parts_mut(
        core::ptr::addr_of_mut!(_sappmem),
        core::ptr::addr_of!(_eappmem) as usize - core::ptr::addr_of!(_sappmem) as usize,
    );

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
