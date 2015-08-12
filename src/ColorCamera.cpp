/**
 * @file ColorCamera.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 14, 2014
 */

#include "../include/pmd_nano/ColorCamera.h"

ColorCamera::ColorCamera() {
}

ColorCamera::~ColorCamera() {
}

cv::Size ColorCamera::colorSize() const {
    throw new UnsupportedException("colorSize");
}

void ColorCamera::start() {
}

void ColorCamera::captureColor(cv::Mat& buffer) {
    throw new UnsupportedException("captureColor");
}
