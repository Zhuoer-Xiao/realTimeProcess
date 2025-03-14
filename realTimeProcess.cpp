#include "realTimeProcess.h"

//���µ�ǰ������ʾ
void updatePointCloudShow() {

}

realTimeProcess::realTimeProcess(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);
    //��ʼ����ص���
    cloudShow.reset(new PointCloudT());
    cloudTar.reset(new PointCloudT());
    cloudSrc.reset(new PointCloudT());
    initialVtkWidget();

    connect(ui.actionloadTarFile, SIGNAL(triggered()), this, SLOT(loadTar()));
    connect(ui.actionloadSrcFile, SIGNAL(triggered()), this, SLOT(loadSrc()));
    connect(ui.enterSystem, SIGNAL(clicked()), this, SLOT(enterSystem()));
    connect(ui.startRegister, SIGNAL(clicked()), this, SLOT(startReg()));
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
    QMessageBox::information(this, "Information", u8"����ϵͳ������");
    //this->close();
    //realSystem* sys = new realSystem();
    //sys->show();
}

void realTimeProcess::startReg()
{
    //QMessageBox::information(this, "Information", u8"��ʼ��׼������");
    Eigen::Matrix4f transform;
    //�����������
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
            QMessageBox::information(this, "Information", u8"��ʼGICP����ȴ��㷨��ɡ�����");
            transform=gicpReg(cloudSrc,cloudTar,flag);
            break;
        case 1:
            QMessageBox::information(this, "Information", u8"��ʼ�Ľ�GICP����ȴ��㷨��ɡ�����");
            transform = multi_scaling_gicp(cloudSrc, cloudTar, flag);
            break;
        case 2:
            QMessageBox::information(this, "Information", u8"��ʼICP��׼������");
            transform=icpReg(cloudSrc, cloudTar, flag);
            break;
        case 3:
            QMessageBox::information(this, "Information", u8"��ʼNICP��׼������");
            transform= normalIcpReg(cloudSrc, cloudTar, flag);
            break;
        case 4:
            QMessageBox::information(this, "Information", u8"��ʼnonlinearICP��׼������");
            transform = nlIcpReg(cloudSrc, cloudTar, flag);
            break;
        case 5:
            QMessageBox::information(this, "Information", u8"��ʼFPFH-SAC��׼������");
            transform = fpfhReg(cloudSrc, cloudTar);
            break;
        case 6:
            QMessageBox::information(this, "Information", u8"��ʼ4pcs��׼������");
            transform = FpcsReg(cloudSrc, cloudTar);
            break;
        case 7:
            QMessageBox::information(this, "Information", u8"��ʼk4pcs��׼������");
            transform = kfpcs(cloudSrc, cloudTar);
            break;
        case 8:
            QMessageBox::information(this, "Information", u8"��ʼNDT��׼������");
            transform = NDT(cloudSrc, cloudTar);
            break;
        default:
            QMessageBox::information(this, "Information", u8"err������");
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
}

void realTimeProcess::clearPointCloud()
{
}

void realTimeProcess::setBackgroundColor()
{
}

void realTimeProcess::saveCurrentPointCloud()
{
}

void realTimeProcess::errorInfo(int errNum)
{
    QString str;
    switch (errNum) {
    case 1:
        str = QStringLiteral("���ƶ�ȡ����");
        break;
    case 2:
        str = QStringLiteral("δ����Դ����");
        break;
    case 3:
        str = QStringLiteral("δ����Ŀ�����");
        break;
    case 4:
        str = QStringLiteral("����·������");
        break;
    case 5:
        str = QStringLiteral("δ����ƴ�ӣ����޽��");
        break;
    case 6:
        str = QStringLiteral("��׼����������ƻ��㷨����");
        break;
    }

    QMessageBox::critical(this, QStringLiteral("����"), str);
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
