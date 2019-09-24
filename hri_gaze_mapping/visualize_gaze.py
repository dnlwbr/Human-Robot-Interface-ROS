import csv
import argparse
import cv2
import numpy as np

from GazeMapper import GazeMapper


def readTimestamps(filename):
    with open(filename, newline='\n') as file:
        reader = csv.DictReader(file, delimiter='\t')
        return [float(row['timestamp']) for row in reader]


def readJournal(filename):
    with open(filename, newline='\n') as file:
        reader = csv.DictReader(file, delimiter='\t')
        return [row for row in reader]


def match(timestamps, source, key_name='timestamp'):
    data = [{'timestamp': ts, 'matches': []} for ts in timestamps]

    current = 0
    lastIdx = len(data)-1
    for row in source:
        ts = float(row[key_name])

        # find first entry equal or larger than ts
        while ts > data[current]['timestamp'] and current < lastIdx:
            current += 1

        timediff = lambda idx: abs(ts - data[idx]['timestamp'])

        # Is current or prev closer to ts?
        prev = max(0, current-1)
        matchedIdx = prev if timediff(prev) < timediff(current) else current
        data[matchedIdx]['matches'].append(row)

    return data


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Overlay gaze on top of field video and map gaze on robot view')

    parser.add_argument('pathToFolder', action='store')
    args = parser.parse_args()
    timestamps = readTimestamps(args.pathToFolder + '/FieldData.tsv')
    journal = readJournal(args.pathToFolder + '/JournalData.tsv')
    matched = match(timestamps, journal, 'sync.timestamp')
    video = cv2.VideoCapture(args.pathToFolder + '/Field.mp4')

    # Check if total number of frames equals number of timestamps
    assert video.get(cv2.CAP_PROP_FRAME_COUNT) == len(timestamps)

    # Starting frame if desired
    video.set(cv2.CAP_PROP_POS_FRAMES, len(timestamps)-1)

    mapper = GazeMapper(args.pathToFolder + '/robot.jpg', video, args.pathToFolder)

    while True:
        ret, frame = video.read()
        if not ret:
            break

        cur_frame_idx = int(video.get(cv2.CAP_PROP_POS_FRAMES)) - 1

        get = lambda name: int(round(np.median([float(entry[name]) for entry in matched[cur_frame_idx]['matches']])))
        x = get('field.gaze.x')
        y = get('field.gaze.y')

        mapper.map(frame, (x, y))
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break

    mapper.release()
