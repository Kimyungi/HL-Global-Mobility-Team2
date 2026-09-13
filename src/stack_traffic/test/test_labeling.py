"""Dataset integrity and annotation coordinate tests."""
import json

import pytest
from PIL import Image

from stack_traffic.labeling import decode_boxes, encode_boxes, prepare


def test_coordinates_roundtrip():
    boxes = [(0, 0, 1280, 720), (432, 23, 501, 79)]
    result = decode_boxes(encode_boxes(boxes, 1280, 720), 1280, 720)
    for actual, expected in zip(result, boxes):
        assert actual == pytest.approx(expected, abs=0.001)
    assert decode_boxes('', 1280, 720) == []


@pytest.mark.parametrize('row', ['1 .5 .5 .1 .1', '0 .5 .5 -1 .1',
                                  '0 1 .5 .2 .2', '0 nan .5 .2 .2'])
def test_bad_labels_rejected(row):
    with pytest.raises(ValueError):
        decode_boxes(row, 1280, 720)


def test_prepare_deduplicates_and_preserves_resume(tmp_path):
    source, dataset = tmp_path / 'raw', tmp_path / 'dataset'
    source.mkdir()
    Image.new('RGB', (32, 20), 'red').save(source / 'a.png')
    (source / 'duplicate.png').write_bytes((source / 'a.png').read_bytes())
    Image.new('RGB', (32, 20), 'blue').save(source / 'b.png')
    prepare(source, dataset, 3000)
    manifest = json.loads((dataset / 'manifest.json').read_text())
    assert len(manifest['images']) == 2
    assert list((dataset / 'labels').iterdir()) == []
    with pytest.raises(ValueError):
        prepare(source, dataset, 3000)
    assert len(list((dataset / 'images').iterdir())) == 2


def test_prepare_normalizes_exif_orientation(tmp_path):
    source, dataset = tmp_path / 'raw', tmp_path / 'dataset'
    source.mkdir()
    pic = Image.new('RGB', (32, 20))
    exif = pic.getexif()
    exif[274] = 6
    pic.save(source / 'rotated.jpg', exif=exif)
    prepare(source, dataset, 3000)
    with Image.open(dataset / 'images' / 'yongin_000001.png') as output:
        assert output.size == (20, 32)


def test_prepare_include_excludes_unrelated_photos(tmp_path):
    source, dataset = tmp_path / 'raw', tmp_path / 'dataset'
    (source / 'traffic_session').mkdir(parents=True)
    (source / 'exit_session').mkdir()
    Image.new('RGB', (32, 20), 'red').save(source / 'traffic_session' / 'a.jpg')
    Image.new('RGB', (32, 20), 'blue').save(source / 'exit_session' / 'b.jpg')
    prepare(source, dataset, 3000, 'traffic_*/*.jpg')
    records = json.loads((dataset / 'manifest.json').read_text())['images']
    assert len(records) == 1
    assert 'traffic_session' in records[0]['source']
