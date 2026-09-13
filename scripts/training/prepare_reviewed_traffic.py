"""Materialize reviewed corrections and retain unflagged automatic labels.

Does not treat untouched automatic labels as human reviewed. Excluded images
are omitted even when a previously approved label file still exists.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from PIL import Image


def prepare(source, output):
    source, output = source.resolve(), output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError('Output must be empty; refusing to overwrite a dataset')
    manifest = json.loads((source / 'manifest.json').read_text())
    decisions = json.loads((source / 'review_decisions.json').read_text())
    review_set = {r['image'] for r in json.loads((source / 'review_queue_594.json').read_text())}
    if review_set - decisions.keys():
        raise ValueError('The requested review queue is incomplete')
    planned = []
    for row in manifest['images']:
        decision = decisions.get(row['image'])
        if decision and decision['status'] == 'excluded':
            continue
        if decision and decision['status'] not in ('approved', 'approved_background'):
            raise ValueError('Unknown decision status')
        image = source / 'images' / row['image']
        label = source / ('reviewed_labels' if decision else 'roboflow_labels_draft') / (image.stem + '.txt')
        with Image.open(image) as pic:
            pic.load()
            width, height = pic.size
        if (width, height) != (row['width'], row['height']):
            raise ValueError('Image dimensions changed')
        lines = label.read_text().splitlines()
        for line in lines:
            values = line.split()
            if len(values) != 5 or values[0] != '0':
                raise ValueError('Invalid traffic light label')
            cx, cy, w, h = map(float, values[1:])
            if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < w <= 1 and 0 < h <= 1):
                raise ValueError('Invalid normalized coordinates')
            if cx-w/2 < -1e-7 or cy-h/2 < -1e-7 or cx+w/2 > 1+1e-7 or cy+h/2 > 1+1e-7:
                raise ValueError('Box lies outside image')
        if decision and decision['status'] == 'approved' and not lines:
            raise ValueError('Approved image has an empty label')
        if decision and decision['status'] == 'approved_background' and lines:
            raise ValueError('Background image has boxes')
        if decision and decision.get('boxes') != len(lines):
            raise ValueError('Saved review count differs from label count')
        planned.append((row, image, label, len(lines), decision))
    if len(planned) != 2726:
        raise ValueError(f'Expected 2726 new images, found {len(planned)}')
    (output / 'images').mkdir(parents=True, exist_ok=True)
    (output / 'labels').mkdir()
    exported = []
    for row, image, label, count, decision in planned:
        name = 'new_' + image.name
        shutil.copy2(image, output / 'images' / name)
        shutil.copy2(label, output / 'labels' / (Path(name).stem + '.txt'))
        exported.append(dict(image=name, original_image=row['image'],
                             original_source=row['original_source'],
                             session=row['session'], crop_xyxy=row['crop_xyxy'],
                             width=row['width'], height=row['height'],
                             label_source=str(label), boxes=count,
                             annotation_status='human_reviewed' if decision else 'automatic_unreviewed',
                             image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
                             label_sha256=hashlib.sha256(label.read_bytes()).hexdigest()))
    result = dict(classes=['traffic light'], roi=manifest['roi'], images=exported)
    (output / 'manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    summary = dict(images=len(exported), boxes=sum(r['boxes'] for r in exported),
                   human_reviewed=sum(r['annotation_status']=='human_reviewed' for r in exported),
                   automatic_unreviewed=sum(r['annotation_status']=='automatic_unreviewed' for r in exported),
                   excluded=sum(d['status']=='excluded' for d in decisions.values()),
                   unique_image_hashes=len({r['image_sha256'] for r in exported}),
                   old_images_needed=1274, target_total=4000, split_assigned=False)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('datasets/yongin_traffic_center'))
    parser.add_argument('--output', type=Path, default=Path('datasets/traffic_training_4000/new_2726'))
    args = parser.parse_args()
    prepare(args.source, args.output)
