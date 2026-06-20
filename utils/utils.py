"""
LaneDetection/utils/utils.py dosyasını üst-düzey utils paketine köprüler.
lane_detection.py'daki `from utils.utils import ...` importlarının çalışmasını sağlar.
"""
from pathlib import Path
import importlib.util as _ilu

_spec = _ilu.spec_from_file_location(
    '_lane_utils',
    str(Path(__file__).parent.parent / 'LaneDetection' / 'utils' / 'utils.py')
)
_lane_utils = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_lane_utils)

# lane_detection.py'ın ihtiyaç duyduğu semboller
select_device      = _lane_utils.select_device
driving_area_mask  = _lane_utils.driving_area_mask
lane_line_mask     = _lane_utils.lane_line_mask
show_seg_result    = _lane_utils.show_seg_result
