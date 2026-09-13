"""Offline manual traffic-light annotation, runnable through ros2 run."""

import argparse
import hashlib
import json
from pathlib import Path
import math

from PIL import Image, ImageOps


EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff'}


def atomic_write(path, text):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(text, encoding='utf-8')
    temporary.replace(path)


def prepare(source, root, limit, include='**/*'):
    """Copy distinct images and preserve an auditable, resumable manifest."""
    source, root = source.resolve(), root.resolve()
    if not source.is_dir():
        raise ValueError(f'사진 폴더가 없습니다: {source}')
    if source == root or source in root.parents or root in source.parents:
        raise ValueError('원본과 데이터셋 폴더는 서로 포함하지 않아야 합니다.')
    manifest = root / 'manifest.json'
    if manifest.exists():
        raise ValueError('이미 준비된 데이터셋입니다. label 명령으로 이어서 작업하세요.')
    if root.exists() and any(root.iterdir()):
        raise ValueError('준비 폴더가 비어 있지 않습니다. 새 폴더를 지정하세요.')
    candidates = sorted(p for p in source.glob(include)
                        if p.is_file() and p.suffix.lower() in EXTENSIONS)
    unique, seen = [], set()
    for path in candidates:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest not in seen:
            seen.add(digest)
            unique.append((path, digest))
    # Spread selection over the sorted sequence instead of taking the first burst.
    if len(unique) > limit:
        unique = [unique[i * len(unique) // limit] for i in range(limit)]
    if not unique:
        raise ValueError('지원하는 사진이 없습니다.')
    # Validate every selected image before creating the dataset.
    for path, _ in unique:
        with Image.open(path) as picture:
            picture.load()
    (root / 'images').mkdir(parents=True)
    (root / 'labels').mkdir()
    records = []
    for i, (path, digest) in enumerate(unique):
        name = f'yongin_{i + 1:06d}.png'
        with Image.open(path) as picture:
            # Bake EXIF orientation into pixels; labels match training pixels.
            picture = ImageOps.exif_transpose(picture).convert('RGB')
            picture.save(root / 'images' / name)
            width, height = picture.size
        records.append(dict(image=name, source=str(path), sha256=digest,
                            width=width, height=height))
    atomic_write(manifest, json.dumps(dict(classes=['traffic light'],
                 target=limit, images=records), ensure_ascii=False, indent=2))
    print(f'준비 완료: {len(records)}/{limit}장 — {root}')


def encode_boxes(boxes, width, height):
    rows = []
    for x1, y1, x2, y2 in boxes:
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError('이미지 범위를 벗어나거나 크기가 0인 박스입니다.')
        rows.append(f'0 {(x1+x2)/2/width:.8f} {(y1+y2)/2/height:.8f} '
                    f'{(x2-x1)/width:.8f} {(y2-y1)/height:.8f}\n')
    return ''.join(rows)


def decode_boxes(text, width, height):
    boxes = []
    for line in text.splitlines():
        cls, cx, cy, w, h = map(float, line.split())
        if not all(math.isfinite(v) for v in (cls, cx, cy, w, h)) or cls != 0 or w <= 0 or h <= 0:
            raise ValueError('잘못된 라벨입니다.')
        boxes.append(((cx-w/2)*width, (cy-h/2)*height,
                      (cx+w/2)*width, (cy+h/2)*height))
    # Serialized rounding may move an edge by fractions of a pixel.
    for box in boxes:
        if box[0] < -0.001 or box[1] < -0.001 or box[2] > width+0.001 or box[3] > height+0.001:
            raise ValueError('라벨 좌표가 이미지 밖에 있습니다.')
    return [tuple(max(0, min(v, width if i % 2 == 0 else height))
                  for i, v in enumerate(box)) for box in boxes]


def annotate(root, review=False, review_queue=None):
    import tkinter as tk
    from tkinter import messagebox
    from PIL import ImageTk

    records = json.loads((root / 'manifest.json').read_text())['images']
    decisions_path = root / 'review_decisions.json'
    decisions = json.loads(decisions_path.read_text()) if review and decisions_path.exists() else {}
    reasons = {}
    if review:
        queue = json.loads((review_queue or root / 'review_queue.json').read_text())
        by_name = {r['image']: r for r in records}
        records = [by_name[q['image']] for q in queue]
        reasons = {q['image']: ', '.join(q['reasons']) or '일반 검토' for q in queue}
        (root / 'reviewed_labels').mkdir(exist_ok=True)
    window = tk.Tk()
    window.title('용인 자동 라벨 검토' if review else '용인 OAK-D 신호등 라벨링')
    state = dict(index=0, boxes=[], dirty=False, scale=1.0, start=None)

    def label_path(record):
        return root / ('reviewed_labels' if review else 'labels') / (Path(record['image']).stem + '.txt')

    def redraw():
        canvas.delete('all')
        pic = state['picture']
        scale = state['scale']
        state['render'] = ImageTk.PhotoImage(pic.resize(
            (max(1, round(pic.width*scale)), max(1, round(pic.height*scale)))))
        canvas.create_image(0, 0, anchor='nw', image=state['render'])
        for number, box in enumerate(state['boxes'], 1):
            canvas.create_rectangle(*(v*scale for v in box), outline='#00ff00', width=2)
            canvas.create_text(box[0]*scale, box[1]*scale, text=str(number), fill='yellow', anchor='sw')
        canvas.configure(scrollregion=canvas.bbox('all'))
        done = (sum(r['image'] in decisions for r in records) if review
                else sum(label_path(r).exists() for r in records))
        status.set(f"{state['index']+1}/{len(records)}  저장 완료 {done}/{len(records)}  "
                   f"박스 {len(state['boxes'])}  {'미저장' if state['dirty'] else ''}  "
                   f"{records[state['index']]['image']}  "
                   f"{reasons.get(records[state['index']]['image'], '')}  "
                   f"{decisions.get(records[state['index']]['image'], {}).get('status', '') if review else ''}")

    def load():
        record = records[state['index']]
        with Image.open(root / 'images' / record['image']) as pic:
            state['picture'] = pic.convert('RGB')
        path = label_path(record)
        if review and not path.exists():
            path = root / 'roboflow_labels_draft' / path.name
        pic = state['picture']
        state['boxes'] = decode_boxes(path.read_text(), pic.width, pic.height) if path.exists() else []
        state['dirty'] = False
        state['scale'] = min(1, 1200/pic.width, 700/pic.height)
        canvas.xview_moveto(0)
        canvas.yview_moveto(0)
        redraw()

    def move(step):
        if state['dirty'] and not messagebox.askyesno('미저장 변경', '변경을 버리고 이동할까요?'):
            return
        state['index'] = max(0, min(len(records)-1, state['index']+step))
        load()

    def save(empty=False):
        if empty:
            if not messagebox.askyesno('배경 사진', '신호등이 없는 사진으로 저장할까요?'):
                return
            state['boxes'] = []
        elif not state['boxes']:
            messagebox.showinfo('박스 없음', '신호등이 없다면 배경 저장 버튼을 사용하세요.')
            return
        pic = state['picture']
        atomic_write(label_path(records[state['index']]), encode_boxes(state['boxes'], pic.width, pic.height))
        if review:
            decisions[records[state['index']]['image']] = {'status': 'approved_background' if empty else 'approved', 'boxes': len(state['boxes'])}
            atomic_write(decisions_path, json.dumps(decisions, ensure_ascii=False, indent=2))
        state['dirty'] = False
        move(1)

    def point(event):
        pic = state['picture']
        return (max(0, min(pic.width, canvas.canvasx(event.x)/state['scale'])),
                max(0, min(pic.height, canvas.canvasy(event.y)/state['scale'])))

    def press(event):
        state['start'] = point(event)

    def drag(event):
        if state['start']:
            canvas.delete('preview')
            canvas.create_rectangle(*(v*state['scale'] for v in (*state['start'], *point(event))),
                                    outline='yellow', tags='preview')

    def release(event):
        if state['start'] is None:
            return
        x1, y1 = state['start']
        x2, y2 = point(event)
        state['start'] = None
        if abs(x2-x1) >= 2 and abs(y2-y1) >= 2:
            state['boxes'].append((min(x1,x2), min(y1,y2), max(x1,x2), max(y1,y2)))
            state['dirty'] = True
        redraw()

    def undo():
        if state['boxes']:
            state['boxes'].pop()
            state['dirty'] = True
            redraw()

    def exclude():
        if not messagebox.askyesno('학습 제외', '가림/흐림 등으로 이 사진을 학습에서 제외할까요?'):
            return
        decisions[records[state['index']]['image']] = {'status': 'excluded'}
        atomic_write(decisions_path, json.dumps(decisions, ensure_ascii=False, indent=2))
        state['dirty'] = False
        move(1)

    def remove_at(event):
        x, y = point(event)
        hits = [(i, (b[2]-b[0])*(b[3]-b[1])) for i,b in enumerate(state['boxes'])
                if b[0] <= x <= b[2] and b[1] <= y <= b[3]]
        if hits:
            index = min(hits, key=lambda h: h[1])[0]
            state['boxes'].pop(index)
            state['dirty'] = True
            state['start'] = None
            redraw()
        return 'break'

    def zoom(factor):
        state['scale'] = max(0.1, min(4, state['scale']*factor))
        redraw()

    def close():
        if not state['dirty'] or messagebox.askyesno('종료', '미저장 변경을 버리고 종료할까요?'):
            window.destroy()

    bar = tk.Frame(window)
    bar.pack(fill='x')
    for title, command in [('이전 [A]', lambda: move(-1)), ('다음 [D]', lambda: move(1)),
                           ('저장+다음 [S]', save), ('배경 저장 [N]', lambda: save(True)),
                           ('마지막 박스 삭제 [Z]', undo), ('확대 [+]', lambda: zoom(1.25)),
                           ('축소 [-]', lambda: zoom(0.8))]:
        tk.Button(bar, text=title, command=command).pack(side='left')
    if review:
        tk.Button(bar, text='학습 제외 [X]', command=exclude).pack(side='left')
        window.bind('<x>', lambda e: exclude())
    status = tk.StringVar()
    tk.Label(window, textvariable=status).pack()
    tk.Label(window, text='왼쪽 드래그: 박스 추가 | Shift+클릭: 해당 박스 삭제 | 오른쪽 드래그: 화면 이동').pack()
    canvas = tk.Canvas(window, width=1200, height=700, bg='#222222')
    canvas.pack(fill='both', expand=True)
    canvas.bind('<ButtonPress-1>', press)
    canvas.bind('<Shift-ButtonPress-1>', remove_at)
    canvas.bind('<B1-Motion>', drag)
    canvas.bind('<ButtonRelease-1>', release)
    canvas.bind('<ButtonPress-3>', lambda e: canvas.scan_mark(e.x, e.y))
    canvas.bind('<B3-Motion>', lambda e: canvas.scan_dragto(e.x, e.y, gain=1))
    for key, command in [('a', lambda: move(-1)), ('d', lambda: move(1)), ('s', save),
                         ('n', lambda: save(True)), ('z', undo),
                         ('plus', lambda: zoom(1.25)), ('minus', lambda: zoom(0.8))]:
        window.bind('<'+key+'>', lambda e, fn=command: fn())
    window.protocol('WM_DELETE_WINDOW', close)
    state['index'] = next((i for i,r in enumerate(records)
                           if (r['image'] not in decisions if review else not label_path(r).exists())), 0)
    load()
    window.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['prepare', 'label', 'review', 'check'])
    parser.add_argument('--dataset', type=Path, default=Path('datasets/yongin_traffic'))
    parser.add_argument('--source', type=Path)
    parser.add_argument('--limit', type=int, default=3000)
    parser.add_argument('--include', default='**/*', help='원본 폴더 기준 사진 glob 패턴')
    parser.add_argument('--review-queue', type=Path, help='검토할 사진 목록 JSON')
    args = parser.parse_args()
    try:
        if args.command == 'prepare':
            if args.source is None or args.limit <= 0:
                parser.error('prepare에는 --source와 양수 --limit가 필요합니다.')
            prepare(args.source, args.dataset, args.limit, args.include)
        elif args.command in ('label', 'review'):
            annotate(args.dataset, review=args.command == 'review',
                     review_queue=args.review_queue)
        else:
            records = json.loads((args.dataset / 'manifest.json').read_text())['images']
            done = 0
            for record in records:
                with Image.open(args.dataset / 'images' / record['image']) as pic:
                    pic.load()
                    path = args.dataset / 'labels' / (Path(record['image']).stem + '.txt')
                    if path.exists():
                        decode_boxes(path.read_text(), pic.width, pic.height)
                        done += 1
            print(f'라벨 검증 통과: 완료 {done}/{len(records)}, 미완료 {len(records)-done}')
    except (OSError, ValueError) as error:
        parser.exit(1, f'오류: {error}\n')


if __name__ == '__main__':
    main()
