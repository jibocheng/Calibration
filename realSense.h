#pragma once
#include <QObject>
#include <QThread>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <QImage>

class realSense : public QObject {
    Q_OBJECT

public:
    explicit realSense(QObject* parent = nullptr);
    ~realSense();

    void initializeCamera();
    void stopCapturing();
    bool isRunning() const { return capturing; }

signals:
    void frame_sender(const QImage& qImage);
    void cameraError(const QString& error);
    void cameraInfo(const QString& info);
    void intrinisic_sender(const rs2_intrinsics intrinsics);

public slots:
    void captureFrames();

private:
    rs2::pipeline pipeline;
    rs2::config cfg;
    rs2::pipeline_profile pipeline_profile;
    rs2_intrinsics device_intrinsics;
    bool capturing = false;
    bool is_device_connected = false;

    QString info_text;
    QString error_message;

private:
    void stopPipeline();
    QImage cvMat2QImage(const cv::Mat& mat);
    QString identifyCameraType();
    void getCameraIntrinsics();
};
