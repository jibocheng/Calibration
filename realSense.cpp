#include "realSense.h"

realSense::realSense(QObject* parent)
    : QObject(parent), capturing(false), is_device_connected(false) {
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
}

realSense::~realSense() {
    stopPipeline();
}

void realSense::initializeCamera() {
    try {
        QString device_name = identifyCameraType();
        if (device_name != "No device") {
            is_device_connected = true;
            emit cameraInfo("RealSense detected successfully: " + device_name);
            pipeline_profile = pipeline.start(cfg);
            getCameraIntrinsics();
        }
        else {
            emit cameraInfo("No RealSense device found.");
        }
    }
    catch (const rs2::error& e) {
        emit cameraError("RealSense error: " + QString::fromStdString(e.what()));
    }
}

void realSense::stopCapturing() {
    capturing = false;
    stopPipeline();
}

void realSense::captureFrames() {
    if (!is_device_connected) {
        emit cameraError("No device connected");
        return;
    }

    capturing = true;
    cv::Mat image;
    try {
        while (capturing) {
            rs2::frameset frames = pipeline.wait_for_frames();
            rs2::frame color_frame = frames.get_color_frame();

            const int w = color_frame.as<rs2::video_frame>().get_width();
            const int h = color_frame.as<rs2::video_frame>().get_height();
            image = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            QImage qImage = cvMat2QImage(image);
            emit frame_sender(qImage);
            QThread::msleep(10);
        }
    }
    catch (const rs2::error& e) {
        emit cameraError("RealSense error during capture: " + QString::fromStdString(e.what()));
    }
}

QString realSense::identifyCameraType() {
    rs2::device_list devices = rs2::context().query_devices();
    if (devices.size() == 0) {
        return "No device";
    }
    rs2::device device = devices[0];
    return QString::fromStdString(device.get_info(RS2_CAMERA_INFO_NAME));
}

void realSense::getCameraIntrinsics() {
    if (!is_device_connected) {
        emit cameraError("Device not connected.");
        return;
    }

    rs2::stream_profile stream_profile = pipeline_profile.get_stream(RS2_STREAM_COLOR);
    rs2::video_stream_profile video_stream_profile = stream_profile.as<rs2::video_stream_profile>();
    device_intrinsics = video_stream_profile.get_intrinsics();
    emit intrinisic_sender(device_intrinsics);
}

void realSense::stopPipeline() {
    if (is_device_connected) {
        try {
            pipeline.stop();
            is_device_connected = false;
        }
        catch (const std::exception& e) {
            emit cameraError("Error stopping pipeline: " + QString::fromStdString(e.what()));
        }
    }
}


QImage realSense::cvMat2QImage(const cv::Mat& mat)
{
    QImage image;
    switch (mat.type())
    {
    case CV_8UC1:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
        break;
    case CV_8UC3:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        image = image.rgbSwapped();
        break;
    case CV_8UC4:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        break;
    case CV_16UC4:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGBA64);
        image = image.rgbSwapped();
        break;
    }
    return image;
}
