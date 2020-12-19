#![feature(const_fn_floating_point_arithmetic)]

use std::{
    convert::TryFrom,
    fmt::Debug,
    mem::size_of,
    ops::{Index, IndexMut},
    slice::{self, Iter, IterMut, SliceIndex},
};
use thiserror::Error;

use esp_idf_sys::{
    c_types::c_void, esp, esp_err_t, gpio_num_t, rmt_carrier_level_t_RMT_CARRIER_LEVEL_LOW,
    rmt_channel_id_t, rmt_config, rmt_config_t, rmt_config_t__bindgen_ty_1, rmt_driver_install,
    rmt_driver_uninstall, rmt_idle_level_t_RMT_IDLE_LEVEL_LOW, rmt_item32_s__bindgen_ty_1,
    rmt_item32_s__bindgen_ty_1__bindgen_ty_1, rmt_item32_t, rmt_mode_t_RMT_MODE_TX,
    rmt_translator_init, rmt_tx_config_t, rmt_wait_tx_done, rmt_write_sample, size_t, Error,
};

pub enum RmtChannel {
    _0,
    _1,
    _2,
    _3,
    _4,
    _5,
    _6,
    _7,
}

pub enum OutputPin {
    _0,
    _1,
    _2,
    _3,
    _4,
    _5,
    _6,
    _7,
    _8,
    _9,
    _10,
    _11,
    _12,
    _13,
    _14,
    _15,
    _16,
    _17,
    _18,
    _19,
    _20,
    _21,
    _22,
    _23,
    _25,
    _26,
    _27,
    _28,
    _29,
    _30,
    _31,
    _32,
    _33,
}

#[derive(Debug, Error)]
#[error("channel {0} is out of range, RMT channel numbers are 0 through 7")]
pub struct ChannelOutOfRange(u32);

#[derive(Debug, Error)]
#[error("pin {0} is out of range, output pin numbers are 0 through 33 excluding 24")]
pub struct PinOutOfRange(u32);

impl TryFrom<u32> for RmtChannel {
    type Error = ChannelOutOfRange;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        use RmtChannel::*;

        Ok(match value {
            0 => _0,
            1 => _1,
            2 => _2,
            3 => _3,
            4 => _4,
            5 => _5,
            6 => _6,
            7 => _7,
            _ => Err(ChannelOutOfRange(value))?,
        })
    }
}

impl TryFrom<u32> for OutputPin {
    type Error = ChannelOutOfRange;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        use OutputPin::*;

        Ok(match value {
            0 => _0,
            1 => _1,
            2 => _2,
            3 => _3,
            4 => _4,
            5 => _5,
            6 => _6,
            7 => _7,
            8 => _8,
            9 => _9,
            10 => _10,
            11 => _11,
            12 => _12,
            13 => _13,
            14 => _14,
            15 => _15,
            16 => _16,
            17 => _17,
            18 => _18,
            19 => _19,
            20 => _20,
            21 => _21,
            22 => _22,
            23 => _23,
            25 => _25,
            26 => _26,
            27 => _27,
            28 => _28,
            29 => _29,
            30 => _30,
            31 => _31,
            32 => _32,
            33 => _33,
            _ => Err(ChannelOutOfRange(value))?,
        })
    }
}

impl RmtChannel {
    fn to_rmt_channel_id_t(&self) -> rmt_channel_id_t {
        use esp_idf_sys::{
            rmt_channel_id_t_RMT_CHANNEL_0, rmt_channel_id_t_RMT_CHANNEL_1,
            rmt_channel_id_t_RMT_CHANNEL_2, rmt_channel_id_t_RMT_CHANNEL_3,
            rmt_channel_id_t_RMT_CHANNEL_4, rmt_channel_id_t_RMT_CHANNEL_5,
            rmt_channel_id_t_RMT_CHANNEL_6, rmt_channel_id_t_RMT_CHANNEL_7,
        };
        use RmtChannel::*;

        match self {
            _0 => rmt_channel_id_t_RMT_CHANNEL_0,
            _1 => rmt_channel_id_t_RMT_CHANNEL_1,
            _2 => rmt_channel_id_t_RMT_CHANNEL_2,
            _3 => rmt_channel_id_t_RMT_CHANNEL_3,
            _4 => rmt_channel_id_t_RMT_CHANNEL_4,
            _5 => rmt_channel_id_t_RMT_CHANNEL_5,
            _6 => rmt_channel_id_t_RMT_CHANNEL_6,
            _7 => rmt_channel_id_t_RMT_CHANNEL_7,
        }
    }
}

impl OutputPin {
    fn to_gpio_num_t(&self) -> gpio_num_t {
        use esp_idf_sys::{
            gpio_num_t_GPIO_NUM_0, gpio_num_t_GPIO_NUM_1, gpio_num_t_GPIO_NUM_10,
            gpio_num_t_GPIO_NUM_11, gpio_num_t_GPIO_NUM_12, gpio_num_t_GPIO_NUM_13,
            gpio_num_t_GPIO_NUM_14, gpio_num_t_GPIO_NUM_15, gpio_num_t_GPIO_NUM_16,
            gpio_num_t_GPIO_NUM_17, gpio_num_t_GPIO_NUM_18, gpio_num_t_GPIO_NUM_19,
            gpio_num_t_GPIO_NUM_2, gpio_num_t_GPIO_NUM_20, gpio_num_t_GPIO_NUM_21,
            gpio_num_t_GPIO_NUM_22, gpio_num_t_GPIO_NUM_23, gpio_num_t_GPIO_NUM_25,
            gpio_num_t_GPIO_NUM_26, gpio_num_t_GPIO_NUM_27, gpio_num_t_GPIO_NUM_28,
            gpio_num_t_GPIO_NUM_29, gpio_num_t_GPIO_NUM_3, gpio_num_t_GPIO_NUM_30,
            gpio_num_t_GPIO_NUM_31, gpio_num_t_GPIO_NUM_32, gpio_num_t_GPIO_NUM_33,
            gpio_num_t_GPIO_NUM_4, gpio_num_t_GPIO_NUM_5, gpio_num_t_GPIO_NUM_6,
            gpio_num_t_GPIO_NUM_7, gpio_num_t_GPIO_NUM_8, gpio_num_t_GPIO_NUM_9,
        };
        use OutputPin::*;

        match self {
            _0 => gpio_num_t_GPIO_NUM_0,
            _1 => gpio_num_t_GPIO_NUM_1,
            _2 => gpio_num_t_GPIO_NUM_2,
            _3 => gpio_num_t_GPIO_NUM_3,
            _4 => gpio_num_t_GPIO_NUM_4,
            _5 => gpio_num_t_GPIO_NUM_5,
            _6 => gpio_num_t_GPIO_NUM_6,
            _7 => gpio_num_t_GPIO_NUM_7,
            _8 => gpio_num_t_GPIO_NUM_8,
            _9 => gpio_num_t_GPIO_NUM_9,
            _10 => gpio_num_t_GPIO_NUM_10,
            _11 => gpio_num_t_GPIO_NUM_11,
            _12 => gpio_num_t_GPIO_NUM_12,
            _13 => gpio_num_t_GPIO_NUM_13,
            _14 => gpio_num_t_GPIO_NUM_14,
            _15 => gpio_num_t_GPIO_NUM_15,
            _16 => gpio_num_t_GPIO_NUM_16,
            _17 => gpio_num_t_GPIO_NUM_17,
            _18 => gpio_num_t_GPIO_NUM_18,
            _19 => gpio_num_t_GPIO_NUM_19,
            _20 => gpio_num_t_GPIO_NUM_20,
            _21 => gpio_num_t_GPIO_NUM_21,
            _22 => gpio_num_t_GPIO_NUM_22,
            _23 => gpio_num_t_GPIO_NUM_23,
            _25 => gpio_num_t_GPIO_NUM_25,
            _26 => gpio_num_t_GPIO_NUM_26,
            _27 => gpio_num_t_GPIO_NUM_27,
            _28 => gpio_num_t_GPIO_NUM_28,
            _29 => gpio_num_t_GPIO_NUM_29,
            _30 => gpio_num_t_GPIO_NUM_30,
            _31 => gpio_num_t_GPIO_NUM_31,
            _32 => gpio_num_t_GPIO_NUM_32,
            _33 => gpio_num_t_GPIO_NUM_33,
        }
    }
}

#[inline(always)]
fn bits(byte: &u8) -> impl Iterator<Item = bool> {
    let byte = *byte;
    (0..8u8).map(move |bit| byte & (1 << (7 - bit)) != 0)
}

#[inline(never)]
extern "C" fn rmt_adapter(
    src: *const c_void,
    dest: *mut rmt_item32_t,
    src_size: size_t,
    wanted_num: size_t,
    translated_size: *mut size_t,
    item_num: *mut size_t,
) {
    let wanted_num = wanted_num / 8;
    if src.is_null() || dest.is_null() {
        unsafe {
            *translated_size = 0;
            *item_num = 0;
        }
    }
    let num = wanted_num.min(src_size);
    let src =
        &unsafe { slice::from_raw_parts(src as *const u8, src_size as usize) }[..num as usize];
    let dest = unsafe { slice::from_raw_parts_mut(dest, (num * 8) as usize) };
    for (bit, item) in src.iter().map(bits).flatten().zip(dest) {
        *item = rmt_item32_t {
            __bindgen_anon_1: rmt_item32_s__bindgen_ty_1 {
                __bindgen_anon_1: rmt_item32_s__bindgen_ty_1__bindgen_ty_1 {
                    _bitfield_1: rmt_item32_s__bindgen_ty_1__bindgen_ty_1::new_bitfield_1(
                        if bit { 12 } else { 4 },
                        1,
                        if bit { 4 } else { 12 },
                        0,
                    ),
                },
            },
        };
    }
    unsafe { *translated_size = num };
    unsafe {
        *item_num = num * 8;
    }
}

pub struct Apa106 {
    channel: RmtChannel,
    buffer: Box<[Color]>,
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Color {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl Color {
    pub fn clear(&mut self) {
        self.red = 0;
        self.green = 0;
        self.blue = 0;
    }
}

impl Apa106 {
    pub fn new(channel: RmtChannel, pin: OutputPin, length: usize) -> Result<Self, Error> {
        let buffer = vec![
            Color {
                green: 0,
                red: 0,
                blue: 0,
            };
            length
        ]
        .into_boxed_slice();
        let config = rmt_config_t {
            channel: channel.to_rmt_channel_id_t(),
            gpio_num: pin.to_gpio_num_t(),
            clk_div: 8,
            mem_block_num: 1,
            rmt_mode: rmt_mode_t_RMT_MODE_TX,
            __bindgen_anon_1: rmt_config_t__bindgen_ty_1 {
                tx_config: rmt_tx_config_t {
                    carrier_freq_hz: 38000,
                    carrier_level: rmt_carrier_level_t_RMT_CARRIER_LEVEL_LOW,
                    idle_level: rmt_idle_level_t_RMT_IDLE_LEVEL_LOW,
                    carrier_duty_percent: 33,
                    carrier_en: false,
                    loop_en: false,
                    idle_output_en: true,
                },
            },
        };
        esp!(unsafe { rmt_config(&config) })?;
        esp!(unsafe { rmt_driver_install(config.channel, 0, 0) })?;
        esp!(unsafe { rmt_translator_init(config.channel, Some(rmt_adapter)) })?;
        Ok(Apa106 { channel, buffer })
    }
    pub fn flush(&self) -> Result<(), Error> {
        let channel = self.channel.to_rmt_channel_id_t();
        esp!(unsafe { rmt_wait_tx_done(channel, 0) })?;
        esp!(unsafe {
            rmt_write_sample(
                channel,
                self.buffer.as_ptr() as *const u8,
                (self.buffer.len() * size_of::<Color>()) as u32,
                false,
            )
        })?;
        Ok(())
    }
    pub fn iter(&self) -> Iter<Color> {
        self.buffer.iter()
    }
    pub fn iter_mut(&mut self) -> IterMut<Color> {
        self.buffer.iter_mut()
    }
}

impl Debug for Apa106 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self.buffer.as_ref())
    }
}

impl<I: SliceIndex<[Color]>> Index<I> for Apa106 {
    type Output = <I as SliceIndex<[Color]>>::Output;

    fn index(&self, index: I) -> &Self::Output {
        &self.buffer[index]
    }
}

impl<I: SliceIndex<[Color]>> IndexMut<I> for Apa106 {
    fn index_mut(&mut self, index: I) -> &mut Self::Output {
        &mut self.buffer[index]
    }
}

impl<'a> IntoIterator for &'a Apa106 {
    type Item = &'a Color;

    type IntoIter = Iter<'a, Color>;

    fn into_iter(self) -> Self::IntoIter {
        self.buffer.iter()
    }
}

impl<'a> IntoIterator for &'a mut Apa106 {
    type Item = &'a mut Color;

    type IntoIter = IterMut<'a, Color>;

    fn into_iter(self) -> Self::IntoIter {
        self.buffer.iter_mut()
    }
}

impl Drop for Apa106 {
    fn drop(&mut self) {
        unsafe { rmt_driver_uninstall(self.channel.to_rmt_channel_id_t()) };
    }
}
