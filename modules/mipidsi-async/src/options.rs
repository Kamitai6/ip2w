//! Display options and configuration types.

/// Model options passed to initialization
#[derive(Clone)]
#[non_exhaustive]
pub struct ModelOptions {
    /// Subpixel order
    pub color_order: ColorOrder,
    /// Initial display orientation
    pub orientation: Orientation,
    /// Whether to invert colors (INVON)
    pub invert_colors: ColorInversion,
    /// Display refresh order
    pub refresh_order: RefreshOrder,
    /// Display size (width, height)
    pub display_size: (u16, u16),
    /// Display offset (x, y)
    pub display_offset: (u16, u16),
}

impl ModelOptions {
    /// Creates model options for the given framebuffer size
    pub fn with_framebuffer_size(framebuffer_size: (u16, u16)) -> Self {
        Self {
            color_order: ColorOrder::default(),
            orientation: Orientation::default(),
            invert_colors: ColorInversion::default(),
            refresh_order: RefreshOrder::default(),
            display_size: framebuffer_size,
            display_offset: (0, 0),
        }
    }

    /// Creates model options with given size and offset
    pub fn with_sizes(display_size: (u16, u16), display_offset: (u16, u16)) -> Self {
        Self {
            color_order: ColorOrder::default(),
            orientation: Orientation::default(),
            invert_colors: ColorInversion::default(),
            refresh_order: RefreshOrder::default(),
            display_size,
            display_offset,
        }
    }

    /// Returns the display size based on current orientation
    pub(crate) fn display_size(&self) -> (u16, u16) {
        if self.orientation.rotation.is_horizontal() {
            self.display_size
        } else {
            (self.display_size.1, self.display_size.0)
        }
    }
}

/// Display orientation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Orientation {
    /// Rotation angle
    pub rotation: Rotation,
    /// Horizontal mirroring
    pub mirrored: bool,
}

impl Orientation {
    /// Create a new orientation with rotation
    pub const fn new() -> Self {
        Self {
            rotation: Rotation::Deg0,
            mirrored: false,
        }
    }

    /// Rotate the orientation
    #[must_use]
    pub const fn rotate(self, rotation: Rotation) -> Self {
        Self {
            rotation,
            mirrored: self.mirrored,
        }
    }

    /// Mirror the orientation horizontally
    #[must_use]
    pub const fn mirror(self) -> Self {
        Self {
            mirrored: !self.mirrored,
            ..self
        }
    }
}

/// Rotation angle
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Rotation {
    /// No rotation (portrait)
    #[default]
    Deg0,
    /// 90° clockwise (landscape)
    Deg90,
    /// 180° (portrait inverted)
    Deg180,
    /// 270° clockwise (landscape inverted)
    Deg270,
}

impl Rotation {
    /// Returns true if landscape orientation
    pub const fn is_horizontal(self) -> bool {
        matches!(self, Rotation::Deg90 | Rotation::Deg270)
    }
}

/// Color inversion setting
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ColorInversion {
    /// Normal colors
    #[default]
    Normal,
    /// Inverted colors
    Inverted,
}

/// Subpixel color order
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ColorOrder {
    /// RGB order
    #[default]
    Rgb,
    /// BGR order
    Bgr,
}

/// Vertical refresh order
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VerticalRefreshOrder {
    /// Top to bottom
    #[default]
    TopToBottom,
    /// Bottom to top
    BottomToTop,
}

impl VerticalRefreshOrder {
    /// Flip the refresh order
    #[must_use]
    pub const fn flip(self) -> Self {
        match self {
            Self::TopToBottom => Self::BottomToTop,
            Self::BottomToTop => Self::TopToBottom,
        }
    }
}

/// Horizontal refresh order
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HorizontalRefreshOrder {
    /// Left to right
    #[default]
    LeftToRight,
    /// Right to left
    RightToLeft,
}

impl HorizontalRefreshOrder {
    /// Flip the refresh order
    #[must_use]
    pub const fn flip(self) -> Self {
        match self {
            Self::LeftToRight => Self::RightToLeft,
            Self::RightToLeft => Self::LeftToRight,
        }
    }
}

/// Display refresh order
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RefreshOrder {
    /// Vertical refresh order
    pub vertical: VerticalRefreshOrder,
    /// Horizontal refresh order
    pub horizontal: HorizontalRefreshOrder,
}

impl RefreshOrder {
    /// Create a new refresh order
    pub const fn new(vertical: VerticalRefreshOrder, horizontal: HorizontalRefreshOrder) -> Self {
        Self { vertical, horizontal }
    }

    /// Flip vertical refresh order
    #[must_use]
    pub const fn flip_vertical(self) -> Self {
        Self {
            vertical: self.vertical.flip(),
            ..self
        }
    }

    /// Flip horizontal refresh order
    #[must_use]
    pub const fn flip_horizontal(self) -> Self {
        Self {
            horizontal: self.horizontal.flip(),
            ..self
        }
    }
}

/// Tearing effect output setting
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TearingEffect {
    /// Disable tearing effect output
    #[default]
    Off,
    /// Output vertical blanking information
    Vertical,
    /// Output horizontal and vertical blanking information
    HorizontalAndVertical,
}

/// Memory mapping from orientation
#[derive(Debug, Clone, Copy)]
pub struct MemoryMapping {
    /// Reverse column order
    pub reverse_columns: bool,
    /// Reverse row order
    pub reverse_rows: bool,
    /// Swap rows and columns
    pub swap_rows_and_columns: bool,
}

impl From<Orientation> for MemoryMapping {
    fn from(orientation: Orientation) -> Self {
        let Orientation { rotation, mirrored } = orientation;
        
        let (reverse_columns, reverse_rows, swap_rows_and_columns) = match rotation {
            Rotation::Deg0 => (mirrored, false, true),
            Rotation::Deg90 => (false, mirrored, false),
            Rotation::Deg180 => (!mirrored, true, true),
            Rotation::Deg270 => (true, !mirrored, false),
        };
        
        Self {
            reverse_columns,
            reverse_rows,
            swap_rows_and_columns,
        }
    }
}