#include "realTimeProcess.h"

realTimeProcess::realTimeProcess(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);
    //初始化相关点云
    cloudShow.reset(new PointCloudT());
    cloudTar.reset(new PointCloudT());
    cloudSrc.reset(new PointCloudT());
    initialVtkWidget();

    connect(ui.actionloadTarFile, SIGNAL(triggered()), this, SLOT(loadTar()));
    connect(ui.actionloadSrcFile, SIGNAL(triggered()), this, SLOT(loadSrc()));
    connect(ui.enterSystem, SIGNAL(clicked()), this, SLOT(enterSystem()));
    connect(ui.startRegister, SIGNAL(clicked()), this, SLOT(startReg()));
    connect(ui.startPostProcess, SIGNAL(clicked()), this, SLOT(startPostProcess()));
    connect(ui.clearPointCLoud, SIGNAL(clicked()), this, SLOT(clearPointCloud()));
    connect(ui.changeBackground, SIGNAL(clicked()), this, SLOT(setBackgroundColor()));
}

realTimeProcess::~realTimeProcess() {}

void realTimeProcess::loadSrc() {

    QString filter = tr("Point Cloud Files (*.ply *.pcd)");
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open PointCloud"), ".",
        filter);
    if (fileName == "") return;

    QString suffix = QFileInfo(fileName).suffix().toLower();
    logger.log("open src file:" + fileName.toStdString() + " suffix:" + suffix.toStdString());
    cloudSrc->clear();
    if (suffix == "ply") {
        pcl::io::loadPLYFile(fileName.toStdString(), *cloudSrc);
    }
    else if (suffix == "pcd") {
        pcl::io::loadPCDFile(fileName.toStdString(), *cloudSrc);
    }
    else {
        errorInfo(1);
        return;
    }

    isLoadSrc = 1;
    updatePointCloudShow();
}

void realTimeProcess::loadTar() {

    QString filter = tr("Point Cloud Files (*.ply *.pcd)");
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open PointCloud"), ".",
        filter);
    if (fileName == "") return;

    QString suffix = QFileInfo(fileName).suffix().toLower();
    logger.log("open src file:" + fileName.toStdString() + " suffix:" + suffix.toStdString());
    cloudTar->clear();
    if (suffix == "ply") {
        pcl::io::loadPLYFile(fileName.toStdString(), *cloudTar);
    }
    else if (suffix == "pcd") {
        pcl::io::loadPCDFile(fileName.toStdString(), *cloudTar);
    }
    else {
        errorInfo(1);
        return;
    }

    isLoadTar = 1;
    updatePointCloudShow();
}

void realTimeProcess::initialVtkWidget() {
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow =
        vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    view.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow,
        "viewer", false));
    view->setupInteractor(ui.openGLWidget->interactor(),
        ui.openGLWidget->renderWindow());
    ui.openGLWidget->setRenderWindow(view->getRenderWindow());
}

void realTimeProcess::enterSystem() {
    QMessageBox::information(this, "Information", u8"进入系统···");
    //this->close();
    //realSystem* sys = new realSystem();
    //sys->show();
}

void realTimeProcess::startReg()
{
    //QMessageBox::information(this, "Information", u8"开始配准···");
    Eigen::Matrix4f transform;
    //检验输入点云
    if(!isLoadSrc){
        errorInfo(2);
        return;
    }
    if (!isLoadTar) {
        errorInfo(3);
        return;
    }
    bool flag = 1;
    int regAlgoNum=ui.selectRegister->currentIndex();
    switch (regAlgoNum) {
        case 0:
            QMessageBox::information(this, "Information", u8"开始GICP，请等待算法完成···");
            transform=gicpReg(cloudSrc,cloudTar,flag);
            break;
        case 1:
            QMessageBox::information(this, "Information", u8"开始改进GICP，请等待算法完成···");
            transform = multi_scaling_gicp(cloudSrc, cloudTar, flag);
            break;
        case 2:
            QMessageBox::information(this, "Information", u8"开始ICP配准···");
            transform=icpReg(cloudSrc, cloudTar, flag);
            break;
        case 3:
            QMessageBox::information(this, "Information", u8"开始NICP配准···");
            transform= normalIcpReg(cloudSrc, cloudTar, flag);
            break;
        case 4:
            QMessageBox::information(this, "Information", u8"开始nonlinearICP配准···");
            transform = nlIcpReg(cloudSrc, cloudTar, flag);
            break;
        case 5:
            QMessageBox::information(this, "Information", u8"开始FPFH-SAC配准···");
            transform = fpfhReg(cloudSrc, cloudTar);
            break;
        case 6:
            QMessageBox::information(this, "Information", u8"开始4pcs配准···");
            transform = FpcsReg(cloudSrc, cloudTar);
            break;
        case 7:
            QMessageBox::information(this, "Information", u8"开始k4pcs配准···");
            transform = kfpcs(cloudSrc, cloudTar);
            break;
        case 8:
            QMessageBox::information(this, "Information", u8"开始NDT配准···");
            transform = NDT(cloudSrc, cloudTar);
            break;
        default:
            QMessageBox::information(this, "Information", u8"err···");
            break;
    }
    if (!flag) {
        errorInfo(6);
        return;
    }
    pcl::transformPointCloud(*cloudSrc, *cloudSrc, transform);
    updatePointCloudShow();
}

void realTimeProcess::startPostProcess()
{
    QMessageBox::information(this, "Information", u8"开始后处理···");
    //检验输入点云
    if (!isLoadSrc) {
        errorInfo(2);
        return;
    }
    int postAlgoNum = ui.selectPostProcess->currentIndex();
    PointCloudT::Ptr res(new PointCloudT);
    switch (postAlgoNum) {
    case 0:
        QMessageBox::information(this, "Information", u8"开始ISS，请等待算法完成···");
        randomSample(cloudSrc,res);
        break;
    case 1:
        QMessageBox::information(this, "Information", u8"开始Random，请等待算法完成···");
        randomSample(cloudSrc, res);
        break;
    case 2:
        QMessageBox::information(this, "Information", u8"开始voxel···");
        voxelSample(cloudSrc, res,0.5);
        break;
    case 3:
        QMessageBox::information(this, "Information", u8"开始curvature···");
        randomSample(cloudSrc, res);
        break;
    case 4:
        QMessageBox::information(this, "Information", u8"meshing···");
        break;
    default:
        QMessageBox::information(this, "Information", u8"err···");
        break;
    }
    cloudSrc=res;
    updatePointCloudShow();
}

void realTimeProcess::clearPointCloud()
{
    cloudSrc->clear();
    cloudTar->clear();
    updatePointCloudShow();
}

void realTimeProcess::setBackgroundColor()
{
    double r = ui.rSlider->value();
    double g = ui.gSlider->value();
    double b = ui.bSlider->value();
    QString color = QString("rgb(%1,%2,%3)").arg(r).arg(g).arg(b);
    QMessageBox::information(this, "Information", color);
    view->setBackgroundColor(r/255, g/255, b/255);
    view->spin();
    ui.openGLWidget->setRenderWindow(view->getRenderWindow());
}

void realTimeProcess::saveCurrentPointCloud()
{
}

void realTimeProcess::errorInfo(int errNum)
{
    QString str;
    switch (errNum) {
    case 1:
        str = QStringLiteral("点云读取错误");
        break;
    case 2:
        str = QStringLiteral("未输入源点云");
        break;
    case 3:
        str = QStringLiteral("未输入目标点云");
        break;
    case 4:
        str = QStringLiteral("保存路径错误");
        break;
    case 5:
        str = QStringLiteral("未进行拼接，暂无结果");
        break;
    case 6:
        str = QStringLiteral("配准错误，请检查点云或算法参数");
        break;
    }

    QMessageBox::critical(this, QStringLiteral("错误"), str);
}

void realTimeProcess::updatePointCloudShow()
{
    view->removePointCloud("cloudSrc");
    view->removePointCloud("cloudTar");
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbSrc(cloudSrc);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbTar(cloudTar);
    view->addPointCloud(cloudSrc, rgbSrc, "cloudSrc");
    view->addPointCloud(cloudTar, rgbTar, "cloudTar");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudSrc");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudTar");
    view->resetCamera();
    view->spin();
    ui.openGLWidget->setRenderWindow(view->getRenderWindow());
}


