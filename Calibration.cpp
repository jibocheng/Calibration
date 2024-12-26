#include "Calibration.h"

#pragma region initial function
Calibration::Calibration(QWidget* parent)
    : QWidget(parent)
{
    p_ui = new Ui::CalibrationClass();
    p_ui->setupUi(this);
    p_boardGenerator = new boardGenerator();
    p_cameraThread = new QThread();


    m_rs.moveToThread(p_cameraThread);
    connect(p_cameraThread, &QThread::started, &m_rs, &realSense::captureFrames);
    // pushbutton handling:
    connect(p_boardGenerator, &boardGenerator::board_img_sender, this, &Calibration::board_show);
    connect(p_ui->generate_button, &QPushButton::clicked, this, &Calibration::generate_press);
    connect(p_ui->save_button, &QPushButton::clicked, this, &Calibration::save_press);
    connect(p_ui->delete_button, &QPushButton::clicked, this, &Calibration::delete_press);
    connect(p_ui->cam_connect_button, &QPushButton::clicked, this, &Calibration::cam_conn_button_press);
    connect(p_ui->cam_open_button, &QPushButton::clicked, this, &Calibration::cam_open_button_press);
    connect(p_ui->capture_button, &QPushButton::clicked, this, &Calibration::captured_button_press);
    connect(p_ui->rob_vec_input, &QPushButton::clicked, this, &Calibration::rob_vec_input_press);
    connect(p_ui->record_button, &QPushButton::clicked, this, &Calibration::record_button_press);
    connect(p_ui->read_data_button, &QPushButton::clicked, this, &Calibration::read_data_press);
    connect(p_ui->dcalculate_button, &QPushButton::clicked, this, &Calibration::direct_calculate_press);
    connect(p_ui->icalculate_button, &QPushButton::clicked, this, &Calibration::indirect_calculate_press);
    connect(p_ui->save_c_data_button, &QPushButton::clicked, this, &Calibration::save_calibration_data_press);
    connect(p_ui->save_m_data_button, &QPushButton::clicked, this, &Calibration::save_Transform_matrix_press);
    connect(p_ui->refresh_button, &QPushButton::clicked, this, &Calibration::refresh_button_press);

    //slot and signal message handling:
    connect(&m_rs, &realSense::cameraError, this, &Calibration::cameraError_rec, Qt::QueuedConnection);
    connect(&m_rs, &realSense::cameraInfo, this, &Calibration::cameraInfo_rec, Qt::QueuedConnection);
    connect(&m_rs, &realSense::intrinisic_sender, this, &Calibration::intrinisic_rec);
    //slot and signal handling:
    connect(&m_rs, &realSense::frame_sender, this, &Calibration::frame_rec, Qt::QueuedConnection);
    //code read slot and signal handling:
}

Calibration::~Calibration()
{
    m_rs.stopCapturing();
    if (p_cameraThread->isRunning()) {
        p_cameraThread->quit();
        p_cameraThread->wait();
    }
    if (p_cameraThread != NULL)
    {
        delete p_cameraThread;
    }
    if (p_ui != NULL)
    {
        delete p_ui;
    }
    if (p_boardGenerator != NULL)
    {
        delete p_boardGenerator;
    }
    if (p_boardImg != NULL)
    {
        delete p_boardImg;
    }
    if (p_dataFile != NULL)
    {
        if (p_dataFile->isOpen())
            p_dataFile->close();

        delete p_dataFile;
    }
    if (p_code_reader != NULL)
    {
        delete p_code_reader;
    }
}
#pragma endregion

#pragma region display
void Calibration::board_show(cv::Mat boardImg)
{
    p_boardImg = new cv::Mat(boardImg.clone());
    QImage Q_board = cvMat2QImage(*p_boardImg);
    QTransform transform;
    transform.rotate(90);
    QImage Q_boardImg = Q_board.transformed(transform);
    QSize display_label_size = p_ui->board_display_label->size();
    QImage scaledImage = Q_boardImg.scaled(display_label_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    p_ui->board_display_label->setPixmap(QPixmap::fromImage(scaledImage));
}

QImage Calibration::cvMat2QImage(const cv::Mat& mat)
{
    QImage image;
    switch (mat.type())
    {
    case CV_8UC1:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
        break;
    case CV_8UC3:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        break;
    case CV_8UC4:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        break;
    case CV_16UC4:
        image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGBA64);
        break;
    }
    return image;
}


cv::Mat Calibration::QImageToCvMat(const QImage& image) {
    switch (image.format()) {
    case QImage::Format_RGB888: {
        cv::Mat mat(image.height(), image.width(), CV_8UC3, const_cast<uchar*>(image.bits()), image.bytesPerLine());
        return mat.clone();
    }
    case QImage::Format_Grayscale8: {
        cv::Mat mat(image.height(), image.width(), CV_8UC1, const_cast<uchar*>(image.bits()), image.bytesPerLine());
        return mat.clone();
    }
    case QImage::Format_RGBA8888: {
        cv::Mat mat(image.height(), image.width(), CV_8UC4, const_cast<uchar*>(image.bits()), image.bytesPerLine());
        return mat.clone();
    }
    default: {
        qWarning("Unsupported QImage format for conversion to cv::Mat!");
        return cv::Mat();
    }
    }
}

void Calibration::frame_rec(const QImage& qImage)
{
    //p_ui->command_text_browser->append("frame_rec called. Image size:" + QString::number(image.cols) + "x" + QString::number(image.rows));
    //QImage displayFrame = cvMat2QImage(image);
    *p_src = QImageToCvMat(qImage);
    //*p_dst = p_code_reader->detector(*p_src, m_camIntrinsics);
    capturedImage = cvMat2QImage(*p_src);
    p_ui->cam_display_label->setPixmap(QPixmap::fromImage(capturedImage).scaled(p_ui->cam_display_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

}

#pragma endregion

#pragma region function button slot
void Calibration::save_press()
{
    QString dir = QFileDialog::getSaveFileName(this, tr("Image saving"), "", tr("Images (*.png *.jpg *.jpeg *.bmp);;All Files (*)"));
    if (dir.isEmpty()) {
        return;
    }
    std::string savePath = dir.toStdString();
    if (p_boardImg == NULL || p_boardImg->empty())
    {
        QMessageBox::warning(this, tr("No chessboard"), tr("Save failed, check if you generate chessboard"));
        return;
    }

    try {
        if (save_type == 1)
        {
            if (cv::imwrite(savePath, *p_boardImg)) {
                QMessageBox::information(this, tr("success"), tr("save successed"));
            }
            else {
                QMessageBox::warning(this, tr("fail"), tr("can't save chessboard, check path"));
            }
        }
        else if (save_type == 2)
        {
            cv::Mat image = QImageToCvMat(capturedImage);
            if (cv::imwrite(savePath, image))
            {
                QMessageBox::information(this, tr("success"), tr("save successed"));
            }
            else {
                QMessageBox::warning(this, tr("fail"), tr("can't save chessboard, check path"));
            }
        }

    }
    catch (const cv::Exception& e) {
        QMessageBox::critical(this, tr("error"), tr("error happen in saving chessboard: ") + QString::fromStdString(e.what()));
    }
}
void Calibration::delete_press()
{
    p_ui->board_display_label->setPixmap(QPixmap());
    p_ui->board_display_label->update();

}
void Calibration::save_calibration_data_press()
{
    QString filePath = QFileDialog::getSaveFileName(nullptr, "Save Calibration Data", "", "Text Files (*.txt);;All Files (*)");
    if (filePath.isEmpty()) {
        QMessageBox::information(nullptr, "No File Selected", "Please select a valid path to save the file.");
        return;
    }

    p_ui->save_data_line->setText(filePath);

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(nullptr, "Error", "Failed to open the file for writing.");
        return;
    }
    QTextStream out(&file);
    if (p_rob_vecs->size() == 0 || p_cam_vecs->size() == 0)
    {
        QMessageBox::critical(nullptr, "Error", "No calibration data!");
        return;
    }

    size_t num_p_hand_data = p_rob_vecs->size();
    size_t num_p_eye_data = p_cam_vecs->size();
    size_t num_r_hand_data = r_rob_vecs->size();
    size_t num_r_eye_data = r_cam_vecs->size();
    
    if (num_p_hand_data != num_p_eye_data || num_r_hand_data != num_r_eye_data || num_p_hand_data != num_r_hand_data || num_p_eye_data != num_r_eye_data)
    {
        QMessageBox::critical(nullptr, "Error", "check the length of camera input and robot input");
        return;
    }

    size_t num = p_rob_vecs->size();

    for (size_t i = 0; i < num; ++i) {
        const auto& p_rob = (*p_rob_vecs)[i];
        const auto& r_rob = (*r_rob_vecs)[i];
        const auto& p_cam = (*p_cam_vecs)[i];
        const auto& r_cam = (*r_cam_vecs)[i];
        out << "hand," << p_rob[0] << "," << p_rob[1] << "," << p_rob[2] << ","
            << r_rob[0] << "," << r_rob[1] << "," << r_rob[2] << "\n";
        out << "eye," << p_cam[0] << "," << p_cam[1] << "," << p_cam[2] << ","
            << r_cam[0] << "," << r_cam[1] << "," << r_cam[2] << "\n";
    }

    file.close();
    QMessageBox::information(nullptr, "Success", "Calibration data successfully saved to: " + filePath);
}
void Calibration::save_Transform_matrix_press()
{
    if (result.empty()) {
        QMessageBox::critical(nullptr, "Error", "No result has been calculated!");
        return;
    }

    QString filePath = QFileDialog::getSaveFileName(nullptr, "Save Transform Matrix", "", "Text Files (*.txt);;All Files (*)");
    if (filePath.isEmpty()) {
        QMessageBox::information(nullptr, "No File Selected", "Please select a valid path to save the file.");
        return;
    }

    p_ui->save_m_data_button->setText(filePath);

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(nullptr, "Error", "Failed to open the file for writing.");
        return;
    }

    QTextStream out(&file);

    if (p_rob_vecs->size() == 0 || p_cam_vecs->size() == 0) {
        QMessageBox::critical(nullptr, "Error", "No calibration data!");
        file.close();
        return;
    }

    for (int i = 0; i < result.rows; ++i) {
        for (int j = 0; j < result.cols; ++j) {
            out << result.at<double>(i, j);
            if (j < result.cols - 1) {
                out << "\t";
            }
        }
        out << "\n"; 
    }

    file.close();
    QMessageBox::information(nullptr, "Success", "Transform matrix successfully saved to: " + filePath);
}
void Calibration::refresh_button_press()
{
    p_cam_vecs->clear();
    p_rob_vecs->clear();
    p_ui->X_line->clear();
    p_ui->Y_line->clear();
    p_ui->Z_line->clear();
    p_ui->RX_line->clear();
    p_ui->RY_line->clear();
    p_ui->RZ_line->clear();
    p_ui->X_rob_line->clear();
    p_ui->Y_rob_line->clear();
    p_ui->Z_rob_line->clear();
    p_ui->RX_rob_line->clear();
    p_ui->RY_rob_line->clear();
    p_ui->RZ_rob_line->clear();
    result.release();
    p_ui->command_text_browser->clear();
}
#pragma endregion

#pragma region camera button slot
void Calibration::cam_conn_button_press() {
    QString conn_status = p_ui->cam_connect_button->text().trimmed();
    if (conn_status == "Connect") {
        p_ui->cam_connect_button->setText("Disconnect");
        m_rs.initializeCamera();
    }
    else if (conn_status == "Disconnect") {
        m_rs.stopCapturing();
        p_ui->cam_connect_button->setText("Connect");
        p_ui->cam_display_label->setPixmap(QPixmap());
        p_ui->cam_display_label->update();
        if (p_code_reader != NULL)
        {
            delete p_code_reader;
        }
    }
}
void Calibration::cam_open_button_press()
{

    if (p_ui->cam_connect_button->text() == "Connect") {
        QMessageBox::warning(this, "Warning", "Please connect the camera first!");
        return;
    }

    if (p_code_reader != NULL)
    {
        QMessageBox::warning(this, "Warning", "Can't open camera twice at the same time!");
    }
    else
    {
        cv::aruco::CharucoBoard predefinedBoard = p_boardGenerator->get_board();
        p_code_reader = new codeReader(predefinedBoard);
        connect(p_code_reader, &codeReader::T_cam_sender, this, &Calibration::T_cam_rec);
        connect(p_code_reader, &codeReader::code_info_sender, this, &Calibration::code_info_rec);

    }

    if (!m_rs.isRunning())
    {
        p_ui->command_text_browser->append("Ready to start camera");
        p_ui->command_text_browser->append("3");
        QThread::msleep(1000);
        p_ui->command_text_browser->append("2");
        QThread::msleep(1000);
        p_ui->command_text_browser->append("1");
        QThread::msleep(1000);
        p_ui->command_text_browser->append("go");
        p_cameraThread->start();
        p_ui->command_text_browser->append("Camera thread started.");
    }
    else
    {
        QMessageBox::warning(this, "Warning", "Camera is already running!");
    }
}
void Calibration::captured_button_press()
{
    save_type = 2;
    *p_dst = p_code_reader->detector(*p_src, m_camIntrinsics);
    if (p_dst->empty())
    {
        p_ui->command_text_browser->append("Nothing detected!");
        return;
    }

    capturedImage = cvMat2QImage(*p_dst);
    p_ui->board_display_label->setPixmap(QPixmap::fromImage(capturedImage).scaled(p_ui->board_display_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    if (rec_flag == 0)
    {
        QMessageBox::critical(this, "error", "detecting failed!");
        return;
    }
    p_ui->X_line->setText(QString::number(p_cam_vec[0], 'f', 3));
    p_ui->Y_line->setText(QString::number(p_cam_vec[1], 'f', 3));
    p_ui->Z_line->setText(QString::number(p_cam_vec[2], 'f', 3));
    p_ui->RX_line->setText(QString::number(r_cam_vec[0], 'f', 3));
    p_ui->RY_line->setText(QString::number(r_cam_vec[1], 'f', 3));
    p_ui->RZ_line->setText(QString::number(r_cam_vec[2], 'f', 3));
}
void Calibration::record_button_press()
{
    p_cam_vec[0] = p_ui->X_line->text().toDouble();
    p_cam_vec[1] = p_ui->Y_line->text().toDouble();
    p_cam_vec[2] = p_ui->Z_line->text().toDouble();
    r_cam_vec[0] = p_ui->RX_line->text().toDouble();
    r_cam_vec[1] = p_ui->RY_line->text().toDouble();
    r_cam_vec[2] = p_ui->RZ_line->text().toDouble();
    p_cam_vecs->emplace_back(p_cam_vec);
    r_cam_vecs->emplace_back(r_cam_vec);
    QString p_cam_vec_str = "camera translation p_cam_vec: (" + QString::number(p_cam_vec[0], 'f', 5) + " ," + QString::number(p_cam_vec[1], 'f', 5)
        + " ," + QString::number(p_cam_vec[2], 'f', 5) + ") has been recorded.";
    p_ui->command_text_browser->append(p_cam_vec_str);
    QString r_cam_vec_str = "camera translation r_cam_vec: (" + QString::number(r_cam_vec[0], 'f', 5) + " ," + QString::number(r_cam_vec[1], 'f', 5)
        + " ," + QString::number(r_cam_vec[2], 'f', 5) + ") has been recorded.";
    p_ui->command_text_browser->append(r_cam_vec_str);
    p_ui->X_line->clear();
    p_ui->Y_line->clear();
    p_ui->Z_line->clear();
    p_ui->RX_line->clear();
    p_ui->RY_line->clear();
    p_ui->RZ_line->clear();

}
#pragma endregion

#pragma region board button slot
void Calibration::generate_press()
{
    int num = p_ui->board_select_combox->currentIndex();
    p_boardGenerator->generate(num);
    save_type = 1;
}
#pragma endregion

#pragma region robot button slot
void Calibration::rob_vec_input_press() {
    QStringList inputs = { "X", "Y", "Z", "RX", "RY", "RZ" };
    std::vector<QLineEdit*> inputFields = {
        p_ui->X_rob_line, p_ui->Y_rob_line, p_ui->Z_rob_line,
        p_ui->RX_rob_line, p_ui->RY_rob_line, p_ui->RZ_rob_line
    };
    std::vector<double> values;

    for (size_t i = 0; i < inputFields.size(); ++i) {
        if (inputFields[i]->text().isEmpty()) {
            QMessageBox::critical(this, tr("Error"), tr("Input %1 is incomplete!").arg(inputs[i]));
            return;
        }
        bool ok;
        double value = inputFields[i]->text().toDouble(&ok);
        if (!ok) {
            QMessageBox::critical(this, tr("Error"), tr("Invalid input in %1!").arg(inputs[i]));
            return;
        }
        values.push_back(value);
    }

    p_rob_vecs->emplace_back(values[0], values[1], values[2]);
    r_rob_vecs->emplace_back(values[3], values[4], values[5]);
    p_ui->command_text_browser->append(
        tr("Robot translation p_rob_vec: (%1, %2, %3) has been recorded.")
        .arg(values[0], 0, 'f', 5).arg(values[1], 0, 'f', 5).arg(values[2], 0, 'f', 5));
    p_ui->command_text_browser->append(
        tr("Robot rotation r_rob_vec: (%1, %2, %3) has been recorded.")
        .arg(values[3], 0, 'f', 5).arg(values[4], 0, 'f', 5).arg(values[5], 0, 'f', 5));

    for (auto* field : inputFields) {
        field->clear();
    }
}

#pragma endregion

#pragma region camera function
void Calibration::T_cam_rec(const cv::Vec3d pvecs, const cv::Vec3d rvecs)
{
    p_cam_vec = pvecs;
    r_cam_vec = rvecs;
    QString str_text = "camera vector : p_vec : (" + QString::number(p_cam_vec[0], 'f', 3) + ","
        + QString::number(p_cam_vec[1], 'f', 3) + ","
        + QString::number(p_cam_vec[2], 'f', 3) + ")" + " and T_vec : ("
        + QString::number(r_cam_vec[0], 'f', 3) + ","
        + QString::number(r_cam_vec[1], 'f', 3) + ","
        + QString::number(r_cam_vec[2], 'f', 3) + ")";
    p_ui->command_text_browser->append(str_text);
}
void Calibration::cameraError_rec(const QString& error)
{
    QMessageBox::critical(this, "Error", error);
}
void Calibration::cameraInfo_rec(const QString& info)
{
    p_ui->command_text_browser->append(info);
}
void Calibration::intrinisic_rec(const rs2_intrinsics intriniscs)
{
    m_camIntrinsics = intriniscs;
    p_ui->command_text_browser->append("Camera Intrinsics:");
    p_ui->command_text_browser->append("Width: " + QString::number(m_camIntrinsics.width));
    p_ui->command_text_browser->append("Height: " + QString::number(m_camIntrinsics.height));
    p_ui->command_text_browser->append("PPX: " + QString::number(m_camIntrinsics.ppx));
    p_ui->command_text_browser->append("PPY: " + QString::number(m_camIntrinsics.ppy));
    p_ui->command_text_browser->append("FX: " + QString::number(m_camIntrinsics.fx));
    p_ui->command_text_browser->append("FY: " + QString::number(m_camIntrinsics.fy));
    for (int i = 0; i < 5; ++i) {
        p_ui->command_text_browser->append("Coeffs[" + QString::number(i) + "]: " + QString::number(m_camIntrinsics.coeffs[i]));
    }
}
#pragma endregion

#pragma region flag function
void Calibration::code_info_rec(const int error)
{
    QString msg = "error code : " + QString::number(error);
    if (error == 41)
    {
        rec_flag = 0;
        p_ui->command_text_browser->append(msg);
    }
    else
    {
        rec_flag = 1;
    }

}
#pragma endregion

#pragma region convert function
cv::Mat_<double> Calibration::vec3dToMat_double(const QString type)
{
    if (p_cam_vecs->size() != r_cam_vecs->size() || p_cam_vecs->size() != p_rob_vecs->size() || p_rob_vecs->size() != r_rob_vecs->size())
    {
        QMessageBox::critical(this, "error", "size of vectors not match!");
    }
    num_points = p_cam_vecs->size();

    if (type == "robot")
    {
        cv::Mat_<double> r_vecs(static_cast<int>(num_points), 6);
        for (size_t i = 0; i < num_points; ++i) {
            const cv::Vec3d& p_rob = (*p_rob_vecs)[i];
            const cv::Vec3d& r_rob = (*r_rob_vecs)[i];

            r_vecs(static_cast<int>(i), 0) = p_rob[0];
            r_vecs(static_cast<int>(i), 1) = p_rob[1];
            r_vecs(static_cast<int>(i), 2) = p_rob[2];
            r_vecs(static_cast<int>(i), 3) = r_rob[0];
            r_vecs(static_cast<int>(i), 4) = r_rob[1];
            r_vecs(static_cast<int>(i), 5) = r_rob[2];
        }
        return r_vecs;
    }
    else if (type == "camera")
    {
        cv::Mat_<double> c_vecs(static_cast<int>(num_points), 6);
        for (size_t i = 0; i < num_points; ++i) {
            const cv::Vec3d& p_cam = (*p_cam_vecs)[i];
            const cv::Vec3d& r_cam = (*r_cam_vecs)[i];

            c_vecs(static_cast<int>(i), 0) = p_cam[0];
            c_vecs(static_cast<int>(i), 1) = p_cam[1];
            c_vecs(static_cast<int>(i), 2) = p_cam[2];
            c_vecs(static_cast<int>(i), 3) = r_cam[0];
            c_vecs(static_cast<int>(i), 4) = r_cam[1];
            c_vecs(static_cast<int>(i), 5) = r_cam[2];
        }
        return c_vecs;
    }

}
#pragma endregion

#pragma region calculation function
void Calibration::direct_calculate_press()
{
    p_calculator = std::make_unique<calculator>();
    cv::Mat_<double> robpose = vec3dToMat_double("robot");
    cv::Mat_<double> campose = vec3dToMat_double("camera");
    if (robpose.size().empty())
    {
        QMessageBox::critical(this, "error", "No input!");
        return;
    }
    if (robpose.size() != campose.size())
    {
        QMessageBox::critical(this,"error", "the size of robot pose and camera pose should be the same!");
        return;
    }
    if (robpose.cols < 3)
    {
        QMessageBox::critical(this, "error", "input data is not enough to get result!");
        return;
    }
    QString rpy = p_ui->rpy_select_combox->currentText();
    std::string rpy_str = rpy.toStdString();
    QString mode = p_ui->comboBox->currentText();

    if (mode == "Eyes In Hand Calibration")
    {
        p_calculator->calculateEIH(campose, robpose, num_points, rpy_str);
        cv::Mat Hcg = p_calculator->getHcg();
        p_ui->command_text_browser->append("Matrix from camera to gripper!");
        result = Hcg;
    }
    else if (mode == "Eyes To Hand Calibration")
    {
        p_calculator->calculateETH(campose, robpose, num_points, rpy_str);
        cv::Mat Hcb = p_calculator->getHcb();
        p_ui->command_text_browser->append("Matrix from camera to base!");
        result = Hcb;
    }
    std::ostringstream oss;
    for (int i = 0; i < result.rows; ++i)
    {
        for (int j = 0; j < result.cols; ++j)
        {
            oss << result.at<double>(i, j);
            if (j < result.cols - 1)
            {
                oss << "\t";
            }
        }
        oss << "\n";
    }
    QString Hc_qstr = QString::fromStdString(oss.str());
    p_ui->command_text_browser->append(Hc_qstr);
    
}
void Calibration::indirect_calculate_press()
{
    if (indcalculate_flag = 0)
    {
        QMessageBox::critical(this, "error", "No file be detected!");
        return;
    }

    direct_calculate_press();
}
#pragma endregion

#pragma region file function
void Calibration::read_data_press()
{
    QString dir = QFileDialog::getOpenFileName(nullptr, "Select Data File", "", "Text Files (*.txt);;All Files (*)");

    if (dir.isEmpty()) {
        QMessageBox::information(nullptr, "No File Selected", "Please select a valid data file.");
        return;
    }
    p_ui->read_data_line->setText(dir);
    p_cam_vecs->clear();
    r_cam_vecs->clear();
    p_rob_vecs->clear();
    r_rob_vecs->clear();
    QFile file(dir);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::critical(nullptr, "Error", "Failed to open the selected file.");
        return;
    }
    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList parts = line.split(',');
        if (parts.size() != 7) {
            QMessageBox::warning(nullptr, "Invalid Format", "The file contains an invalid line: " + line);
            continue;
        }
        QString label = parts[0];
        bool ok = false;
        double x = parts[1].toDouble(&ok);
        double y = parts[2].toDouble(&ok);
        double z = parts[3].toDouble(&ok);
        double rx = parts[4].toDouble(&ok);
        double ry = parts[5].toDouble(&ok);
        double rz = parts[6].toDouble(&ok);

        if (!ok) {
            QMessageBox::warning(nullptr, "Conversion Error", "Failed to parse a value in line: " + line);
            continue;
        }

        if (label == "eye") {
            p_cam_vecs->emplace_back(x, y, z);
            r_cam_vecs->emplace_back(rx, ry, rz);
        }
        else if (label == "hand") {
            p_rob_vecs->emplace_back(x, y, z);
            r_rob_vecs->emplace_back(rx, ry, rz);
        }
        else {
            QMessageBox::warning(nullptr, "Unknown Label", "Unknown label in line: " + line);
        }
    }
    file.close();
    QMessageBox::information(nullptr, "Success", "Data successfully loaded!");
    indcalculate_flag = 1;

}
#pragma endregion
