#!/usr/bin/env python

import csv
import os
import os.path


CONFIG_FILE_NAME = "sdl_field.map"
PATH_USER_FOLDER = os.path.join(
    os.path.expanduser("~"), "pire_ws", "src", "pire-robosim", "config", CONFIG_FILE_NAME)
PATH_GLOBAL = os.path.join(
    "/", "pire", "config", CONFIG_FILE_NAME)


def get_field_info():

    if os.path.lexists(PATH_USER_FOLDER):
        return _get_field_info(PATH_USER_FOLDER)
    elif os.path.lexists(PATH_GLOBAL):
        return _get_field_info(PATH_GLOBAL)
    else:
        raise IOError("Config file does not exist. Expecting file in one of these locations:\n"
            + PATH_USER_FOLDER + "\n" + PATH_GLOBAL)


def _get_field_info(path):

    matrix = []
    with open(path, "r") as sdl_field_map:
        reader = csv.reader(sdl_field_map)
        for line_list in reader:
            matrix.append([x for x in line_list])

    if not all([len(x) == len(matrix[0]) for x in matrix]):
        raise IOError("Inconsistent sdl map! Check file at: {}".format(path))

    obstacles = []

    # y -v, x ->
    for y in range(0, len(matrix)):
        for x in range(0, len(matrix[y])):
            # skip turtles ('#id')
            if len(matrix[y][x]) > 1:
                continue

            # obstacles are '1'
            if int(matrix[y][x]) == 1:
                obstacles.append((x, y))

    return (len(matrix), len(matrix[0]), obstacles)
