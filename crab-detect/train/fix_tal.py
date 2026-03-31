import subprocess
import re

result = subprocess.run(["find", "venv", "-name", "tal.py"], capture_output=True, text=True)
tal_path = result.stdout.strip().split("\n")[0]

if not tal_path:
    print("tal.py not found!")
    exit(1)

print(f"Found: {tal_path}")

with open(tal_path, "r") as f:
    content = f.read()

old1 = "        bbox_scores[mask_gt] = pd_scores[ind[0], :, ind[1]][mask_gt]"
new1 = "        ind[1] = ind[1].clamp(0, pd_scores.shape[2] - 1)  # MPS fix\n        bbox_scores[mask_gt] = pd_scores[ind[0], :, ind[1]][mask_gt]"

old2 = "        gt_boxes = gt_bboxes.unsqueeze(2).expand(-1, -1, na, -1)[mask_gt]"
new2 = "        _expanded = gt_bboxes.unsqueeze(2).expand(-1, -1, na, -1)  # MPS fix\n        _mask = mask_gt & (mask_gt.cumsum(dim=-1) <= _expanded.shape[2])\n        gt_boxes = _expanded[mask_gt]"

if old1 in content and new1 not in content:
    content = content.replace(old1, new1)
    print("Patch 1 applied")
else:
    print("Patch 1 already applied or line differs")

if old2 in content and new2 not in content:
    content = content.replace(old2, new2)
    print("Patch 2 applied")
else:
    print("Patch 2 already applied or line differs")

with open(tal_path, "w") as f:
    f.write(content)

print("Done")