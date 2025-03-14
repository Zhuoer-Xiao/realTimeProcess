#include "realTimeProcess.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <QFileDialog>

realTimeProcess::realTimeProcess(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);

    initialVtkWidget();

    connect(ui.actionloadTarFile, SIGNAL(triggered()), this, SLOT(onOpen()));
    connect(ui.enterSystem, SIGNAL(triggered()), this, SLOT(enterSystem()));
}

realTimeProcess::~realTimeProcess() {}

void realTimeProcess::onOpen() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".",
        "Open PLY files(*.ply)");
    if (fileName == "") return;
    pcl::io::loadPLYFile(fileName.toStdString(), *cloud);
    view->addPointCloud(cloud, "cloud");
    view->resetCamera();
    view->spin();
    ui.openGLWidget->update();
    QString dir = QFileDialog::getExistingDirectory(
        nullptr,
        u8"ѡ���ļ���", // �Ի������
        "/home", // ��ѡ��ʼĿ¼�����Ը�����Ҫ���Ļ�����
        QFileDialog::ShowDirsOnly // ѡ�ֻ��ʾĿ¼
    );
    QMessageBox msgBox;
    dir = u8"���ڴ����ļ���\n:" + dir;
    msgBox.setText(dir);
    msgBox.setStandardButtons(QMessageBox::Cancel);
    msgBox.exec();
    QMessageBox::information(this, "Information", u8"��ʼ���к���\n������������");
    QMessageBox::information(this, "Information", u8"��ʼ���к���\nģ���ؽ�������");
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
    this->close();
    realSystem* sys = new realSystem();
    sys->show();

}