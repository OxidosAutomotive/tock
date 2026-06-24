// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Components for the Screen.
//!
//! Buffer Size
//! -----------
//!
//! Displays can receive a large amount of data and having larger transfer buffers
//! optimizes the number of bus writes.
//!
//! As memory is limited on some MCUs, the `components::screen_buffer_size``
//! macro allows users to define the size of the screen buffer.
//!
//! Usage
//! -----
//!
//! // Screen
//! ```rust
//! let screen =
//!     components::screen::ScreenComponent::new(board_kernel, tft, None)
//!         .finalize(components::screen_component_static!(40960));
//! ```
//!
//! // Screen with Setup
//! ```rust
//! let screen =
//!     components::screen::ScreenComponent::new(board_kernel, tft, Some(tft))
//!         .finalize(components::screen_component_static!(40960));
//! ```

use capsules_core::virtualizers::selection_policy::RoundRobinPolicy;
use capsules_core::virtualizers::selection_policy::SelectionPolicy;
use capsules_extra::screen::screen::Screen;
use capsules_extra::screen::screen_shared::ScreenShared;
use capsules_extra::virtualizers::screen::virtual_screen_split;
use capsules_extra::virtualizers::screen::virtual_screen_split::ScreenSplitUser;
use core::mem::MaybeUninit;
use kernel::capabilities;
use kernel::component::Component;
use kernel::create_capability;
use kernel::hil;

#[macro_export]
macro_rules! screen_split_mux_component_static {
    ($S:ty, $SP:ty $(,)?) => {{
        kernel::static_buf!(
            capsules_extra::virtualizers::screen::virtual_screen_split::ScreenSplitMux<
                'static,
                $S,
                $SP,
            >
        )
    };};
    ($S:ty $(,)?) => {{
        kernel::static_buf!(
            capsules_extra::virtualizers::screen::virtual_screen_split::ScreenSplitMux<'static, $S>
        )
    };};
}

#[macro_export]
macro_rules! screen_split_user_component_static {
    ($S:ty, $SP:ty $(,)?) => {{
        kernel::static_buf!(
            capsules_extra::virtualizers::screen::virtual_screen_split::ScreenSplitUser<
                'static,
                $S,
                $SP,
            >
        )
    };};
    ($S:ty $(,)?) => {{
        kernel::static_buf!(
            capsules_extra::virtualizers::screen::virtual_screen_split::ScreenSplitUser<
                'static,
                $S,
            >
        )
    };};
}

pub type ScreenSplitMuxComponentType<S, SP> = virtual_screen_split::ScreenSplitMux<'static, S, SP>;

pub struct ScreenSplitMuxComponent<
    S: hil::screen::Screen<'static> + 'static,
    SP: 'static + SelectionPolicy<&'static ScreenSplitUser<'static, S, SP>> = RoundRobinPolicy,
> {
    screen: &'static S,
    selection_policy: SP,
}

impl<S: hil::screen::Screen<'static>> ScreenSplitMuxComponent<S> {
    pub fn new(screen: &'static S) -> Self {
        Self {
            screen,
            selection_policy: RoundRobinPolicy::default(),
        }
    }
}

impl<
        S: hil::screen::Screen<'static>,
        SP: 'static + SelectionPolicy<&'static ScreenSplitUser<'static, S, SP>>,
    > ScreenSplitMuxComponent<S, SP>
{
    pub fn new_with_policy(screen: &'static S, policy: SP) -> Self {
        Self {
            screen,
            selection_policy: policy,
        }
    }
}

impl<
        S: hil::screen::Screen<'static> + 'static,
        SP: 'static + SelectionPolicy<&'static ScreenSplitUser<'static, S, SP>>,
    > Component for ScreenSplitMuxComponent<S, SP>
{
    type StaticInput =
        &'static mut MaybeUninit<virtual_screen_split::ScreenSplitMux<'static, S, SP>>;
    type Output = &'static virtual_screen_split::ScreenSplitMux<'static, S, SP>;

    fn finalize(self, static_input: Self::StaticInput) -> Self::Output {
        let mux = static_input.write(virtual_screen_split::ScreenSplitMux::new_with_policy(
            self.screen,
            self.selection_policy,
        ));

        kernel::hil::screen::Screen::set_client(self.screen, mux);
        kernel::deferred_call::DeferredCallClient::register(mux);

        mux
    }
}

pub type ScreenSplitUserComponentType<S, SP> =
    virtual_screen_split::ScreenSplitUser<'static, S, SP>;

pub struct ScreenSplitUserComponent<
    S: hil::screen::Screen<'static> + 'static,
    SP: 'static + SelectionPolicy<&'static ScreenSplitUser<'static, S, SP>>,
> {
    mux: &'static virtual_screen_split::ScreenSplitMux<'static, S, SP>,
    x: usize,
    y: usize,
    width: usize,
    height: usize,
}

impl<
        S: hil::screen::Screen<'static>,
        SP: 'static + SelectionPolicy<&'static ScreenSplitUser<'static, S, SP>>,
    > ScreenSplitUserComponent<S, SP>
{
    pub fn new(
        mux: &'static virtual_screen_split::ScreenSplitMux<'static, S, SP>,
        x: usize,
        y: usize,
        width: usize,
        height: usize,
    ) -> Self {
        Self {
            mux,
            x,
            y,
            width,
            height,
        }
    }
}

impl<
        S: hil::screen::Screen<'static> + 'static,
        SP: 'static + SelectionPolicy<&'static ScreenSplitUser<'static, S, SP>>,
    > Component for ScreenSplitUserComponent<S, SP>
{
    type StaticInput =
        &'static mut MaybeUninit<virtual_screen_split::ScreenSplitUser<'static, S, SP>>;
    type Output = &'static virtual_screen_split::ScreenSplitUser<'static, S, SP>;

    fn finalize(self, static_input: Self::StaticInput) -> Self::Output {
        let split = static_input.write(virtual_screen_split::ScreenSplitUser::new(
            self.mux,
            self.x,
            self.y,
            self.width,
            self.height,
        ));

        split.add_to_mux();

        split
    }
}

#[macro_export]
macro_rules! screen_component_static {
    ($s:literal $(,)?) => {{
        let buffer = kernel::static_buf!([u8; $s]);
        let screen = kernel::static_buf!(capsules_extra::screen::screen::Screen);

        (buffer, screen)
    };};
}

pub type ScreenComponentType = capsules_extra::screen::screen::Screen<'static>;

pub struct ScreenComponent<const SCREEN_BUF_LEN: usize> {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    screen: &'static dyn kernel::hil::screen::Screen<'static>,
    screen_setup: Option<&'static dyn kernel::hil::screen::ScreenSetup<'static>>,
}

impl<const SCREEN_BUF_LEN: usize> ScreenComponent<SCREEN_BUF_LEN> {
    pub fn new(
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
        screen: &'static dyn kernel::hil::screen::Screen,
        screen_setup: Option<&'static dyn kernel::hil::screen::ScreenSetup>,
    ) -> ScreenComponent<SCREEN_BUF_LEN> {
        ScreenComponent {
            board_kernel,
            driver_num,
            screen,
            screen_setup,
        }
    }
}

impl<const SCREEN_BUF_LEN: usize> Component for ScreenComponent<SCREEN_BUF_LEN> {
    type StaticInput = (
        &'static mut MaybeUninit<[u8; SCREEN_BUF_LEN]>,
        &'static mut MaybeUninit<Screen<'static>>,
    );
    type Output = &'static Screen<'static>;

    fn finalize(self, static_input: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);
        let grant_screen = self.board_kernel.create_grant(self.driver_num, &grant_cap);

        let buffer = static_input.0.write([0; SCREEN_BUF_LEN]);

        let screen = static_input.1.write(Screen::new(
            self.screen,
            self.screen_setup,
            buffer,
            grant_screen,
        ));

        kernel::hil::screen::Screen::set_client(self.screen, screen);
        if let Some(screen_setup) = self.screen_setup {
            kernel::hil::screen::ScreenSetup::set_client(screen_setup, screen);
        }

        screen
    }
}

#[macro_export]
macro_rules! screen_shared_component_static {
    ($s:literal, $S:ty $(,)?) => {{
        let buffer = kernel::static_buf!([u8; $s]);
        let screen = kernel::static_buf!(capsules_extra::screen::screen_shared::ScreenShared<$S>);

        (buffer, screen)
    };};
}

pub type ScreenSharedComponentType<S> =
    capsules_extra::screen::screen_shared::ScreenShared<'static, S>;

pub struct ScreenSharedComponent<
    const SCREEN_BUF_LEN: usize,
    S: hil::screen::Screen<'static> + 'static,
> {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    screen: &'static S,
    apps_regions: &'static [capsules_extra::screen::screen_shared::AppScreenRegion],
}

impl<const SCREEN_BUF_LEN: usize, S: hil::screen::Screen<'static>>
    ScreenSharedComponent<SCREEN_BUF_LEN, S>
{
    pub fn new(
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
        screen: &'static S,
        apps_regions: &'static [capsules_extra::screen::screen_shared::AppScreenRegion],
    ) -> ScreenSharedComponent<SCREEN_BUF_LEN, S> {
        ScreenSharedComponent {
            board_kernel,
            driver_num,
            screen,
            apps_regions,
        }
    }
}

impl<const SCREEN_BUF_LEN: usize, S: hil::screen::Screen<'static>> Component
    for ScreenSharedComponent<SCREEN_BUF_LEN, S>
{
    type StaticInput = (
        &'static mut MaybeUninit<[u8; SCREEN_BUF_LEN]>,
        &'static mut MaybeUninit<ScreenShared<'static, S>>,
    );
    type Output = &'static ScreenShared<'static, S>;

    fn finalize(self, static_input: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);
        let grant_screen = self.board_kernel.create_grant(self.driver_num, &grant_cap);

        let buffer = static_input.0.write([0; SCREEN_BUF_LEN]);

        let screen = static_input.1.write(ScreenShared::new(
            self.screen,
            grant_screen,
            buffer,
            self.apps_regions,
        ));

        kernel::hil::screen::Screen::set_client(self.screen, screen);

        screen
    }
}
