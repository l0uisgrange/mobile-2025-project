# —————————————————————————————————————————————————
# General
# —————————————————————————————————————————————————

WINDOW_NAME = "mobile"
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 600
CAMERA_FPS = 30

# —————————————————————————————————————————————————
# Vision
# —————————————————————————————————————————————————

# Filtering
VISION_TRESH = 100
VISION_TRESH_MAX = 255
VISION_FILTER_DIAM = 9
VISION_FILTER_SIGMA_COLOR = 100
VISION_FILTER_SIGMA_SPACE = 9
# Markers
VISION_MARKERS = [1, 2, 3, 4]
VISION_MARKERS_PADDING = 2
VISION_ROBOT_MARKER = 5
VISION_TARGET_MARKER = 6
# Obstacles
VISION_OBSTACLE_MARGIN = 5
# Aruco
ARUCO_WIDTH = 1000
ARUCO_HEIGHT = 800
# Status bar
STATUS_BAR_HEIGHT = 160
STATUS_INDICATOR_SIZE = 30
STATUS_BAR_SPACING = (800, 50)
STATUS_BAR_LABEL_GAP = 20
STATUS_BAR_FONTSCALE = 1
STATUS_BAR_FONTTHICKNESS = 2

# —————————————————————————————————————————————————
# Styling
# —————————————————————————————————————————————————

GRID_RESOLUTION = 71
GRID_SHAPE = (GRID_RESOLUTION, int(GRID_RESOLUTION * 1.5))
GRID_COLOR = (230, 230, 230)
GRID_THICKNESS = 2

TEXT_TOP_MARGIN = 60
TEXT_SIZE = 2
TEXT_PADDING = 40
TEXT_GRAY_POSITION = 150
TEXT_DELTA = 5

# —————————————————————————————————————————————————
# Colors
# —————————————————————————————————————————————————

COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (255, 255, 255)
COLOR_GRAY = (150, 150, 150)
COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)

# —————————————————————————————————————————————————
# Navigation
# —————————————————————————————————————————————————

CELL_VOID = 0
CELL_OBSTACLE = 1
CELL_ROBOT = 2
CELL_MARGIN = 3
CELL_TARGET = 4

X = 0
Y = 1
WEIGHT = 1
