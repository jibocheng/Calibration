#pragma once
#include "realSense.h"
#include <QtWidgets/QWidget>
#include "ui_Calibration.h"
#include <QtWidgets/QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include "boardGenerator.h"
#include <opencv2/opencv.hpp>
#include "codeReader.h"
#include "calculator.h"

class Calibration : public QWidget
{
    Q_OBJECT

public:
    Calibration(QWidget *parent = nullptr);
    ~Calibration();

signals:

private slots:
    void generate_press();
    void save_press();
    void delete_press();
    void cam_conn_button_press();
    void cam_open_button_press();
    void captured_button_press();
    void record_button_press();
    void rob_vec_input_press();
    void direct_calculate_press();
    void read_data_press();
    void indirect_calculate_press();
    void save_calibration_data_press();
    void save_Transform_matrix_press();
    void refresh_button_press();

private slots:
    void board_show(cv::Mat boardImg);
    void frame_rec(const QImage& qImage);

private slots://message receiver
    void cameraError_rec(const QString& error);
    void cameraInfo_rec(const QString& info);
    void intrinisic_rec(const rs2_intrinsics intriniscs);
    void T_cam_rec(const cv::Vec3d pvecs, const cv::Vec3d rvecs);
    void code_info_rec(const int error);
    


private:
    cv::Mat_<double> vec3dToMat_double(const QString type);

private:
    //camera part 
    rs2_intrinsics m_camIntrinsics;
    std::unique_ptr<std::vector<cv::Vec3d>> p_cam_vecs = std::make_unique<std::vector<cv::Vec3d>>();
    std::unique_ptr<std::vector<cv::Vec3d>> r_cam_vecs = std::make_unique<std::vector<cv::Vec3d>>();
    cv::Vec3d p_cam_vec;
    cv::Vec3d r_cam_vec;
    //robot part
    std::unique_ptr<std::vector<cv::Vec3d>> p_rob_vecs = std::make_unique<std::vector<cv::Vec3d>>();
    std::unique_ptr<std::vector<cv::Vec3d>> r_rob_vecs = std::make_unique<std::vector<cv::Vec3d>>();
    // calculator
    std::unique_ptr<calculator> p_calculator;

private:
    //about image
    QImage capturedImage;
    std::unique_ptr<cv::Mat> p_src = std::make_unique<cv::Mat>();
    std::unique_ptr<cv::Mat> p_dst = std::make_unique<cv::Mat>();
    // other
    Ui::CalibrationClass * p_ui = NULL;
    boardGenerator* p_boardGenerator;
    cv::Mat* p_boardImg;
    QFile* p_dataFile = NULL;
    //std::unique_ptr<realSense> p_rs;
    realSense m_rs;
    QThread* p_cameraThread;
    int save_type = 0;
    // charuco code detection:
    codeReader* p_code_reader;
    int rec_flag = 0;
    bool indcalculate_flag = 0;
    //operation:
    int num_points = 0;
    //result
    cv::Mat result;




//function
private:
    QImage cvMat2QImage(const cv::Mat& mat);
    cv::Mat QImageToCvMat(const QImage& image);
    
};
