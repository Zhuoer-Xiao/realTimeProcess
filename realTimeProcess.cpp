#include "realTimeProcess.h"

realTimeProcess::realTimeProcess(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);
    //��ʼ����ص���
    cloudShow.reset(new PointCloudT());
    cloudTar.reset(new PointCloudT());
    cloudSrc.reset(new PointCloudT());
    cloudShowDown.reset(new PointCloudT());
    initialVtkWidget();

    //�������
    connect(ui.actionloadTarFile, SIGNAL(triggered()), this, SLOT(loadTar()));
    connect(ui.actionloadSrcFile, SIGNAL(triggered()), this, SLOT(loadSrc()));
    connect(ui.loadSrcCloud, SIGNAL(clicked()), this, SLOT(loadSrc()));
    connect(ui.loadTarCloud, SIGNAL(clicked()), this, SLOT(loadTar()));

    //��ǰ�༭�ĵ���
    connect(ui.editSrcCloud, SIGNAL(clicked()), this, SLOT(editSrcCloud()));
    connect(ui.editTarCloud, SIGNAL(clicked()), this, SLOT(editTarCloud()));

    //�ϲ�����
    connect(ui.mergeCloud, SIGNAL(clicked()), this, SLOT(mergeCloud()));

    connect(ui.enterSystem, SIGNAL(clicked()), this, SLOT(enterSystem()));
    connect(ui.startRegister, SIGNAL(clicked()), this, SLOT(startReg()));
    connect(ui.startPostProcess, SIGNAL(clicked()), this, SLOT(startPostProcess()));
    connect(ui.clearPointCloud, SIGNAL(clicked()), this, SLOT(clearPointCloud()));
    connect(ui.changeBackground, SIGNAL(clicked()), this, SLOT(setBackgroundColor()));
    
    connect(ui.enterSystem, SIGNAL(clicked()), this, SLOT(realTimeSys()));
    connect(ui.stopSystem, SIGNAL(clicked()), this, SLOT(stopSystem()));
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
    //QMessageBox::information(this, "Information", u8"����ϵͳ������");
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
    case 0: {
            gicpInputDialog dialog;
            dialog.exec();
            //QString infos = QString("%1,%2,%3,%4").arg(dialog.getMaxDistance()).arg(dialog.getScale()).arg(dialog.getMaxDistance()).arg(dialog.getReciprocalCorrespondences());
            //QMessageBox::information(this, "Information", infos);
            transform = gicpReg(cloudSrc, cloudTar, flag);
            break;
        }
    case 1: {
            //QMessageBox::information(this, "Information", u8"��ʼ�Ľ�GICP����ȴ��㷨��ɡ�����");
            myGicpInputDialog dialog;
            dialog.exec();
            transform = multi_scaling_gicp(cloudSrc, cloudTar, flag);
            break;
        }
            
    case 2: {
            //QMessageBox::information(this, "Information", u8"��ʼICP��׼������");
            icpInputDialog dialog;
            dialog.exec();
            transform = icpReg(cloudSrc, cloudTar, flag);
            break;
        }
            
    case 3: {
            //QMessageBox::information(this, "Information", u8"��ʼNICP��׼������");
            nicpInputDialog dialog;
            dialog.exec();
            transform = normalIcpReg(cloudSrc, cloudTar, flag);
            break;
        }
            
    case 4: {
        //QMessageBox::information(this, "Information", u8"��ʼnonlinearICP��׼������");
        nonlinearIcpInputDialog dialog;
        dialog.exec();
        transform = nlIcpReg(cloudSrc, cloudTar, flag);
        break;
        }
            
    case 5: {
        //QMessageBox::information(this, "Information", u8"��ʼFPFH-SAC��׼������");
        fpfhInputDialog dialog;
        dialog.exec();
        transform = fpfhReg(cloudSrc, cloudTar);
        break;
        }
            
    case 6: {
        //QMessageBox::information(this, "Information", u8"��ʼ4pcs��׼������");
        fpcsInputDialog dialog;
        dialog.exec();
        transform = FpcsReg(cloudSrc, cloudTar);
        break;
        }
            
    case 7: {
        //QMessageBox::information(this, "Information", u8"��ʼk4pcs��׼������");
        kfpcsInputDialog dialog;
        dialog.exec();
        transform = kfpcs(cloudSrc, cloudTar);
        break;
        }
            
    case 8: {
        //QMessageBox::information(this, "Information", u8"��ʼNDT��׼������");
        NDTInputDialog dialog;
        dialog.exec();
        transform = NDT(cloudSrc, cloudTar);
        break;
        }
            
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
    //����У��
    PointCloudT::Ptr cloudToProcess(new PointCloudT);
    if (editFlag) {
        if (!isLoadTar)
        {
            errorInfo(3);
            return;
        }
        cloudToProcess = cloudTar;
    }
    else {
        if (!isLoadSrc)
        {
            errorInfo(2);
            return;
        }
        cloudToProcess = cloudSrc;
    }


    QMessageBox::information(this, "Information", u8"��ʼ��������");
    //�����������
    if (!isLoadSrc) {
        errorInfo(2);
        return;
    }
    int postAlgoNum = ui.selectPostProcess->currentIndex();
    PointCloudT::Ptr res(new PointCloudT);
    switch (postAlgoNum) {
    case 0:
    {
        //QMessageBox::information(this, "Information", u8"��ʼISS����ȴ��㷨��ɡ�����");
        issInputDialog dialog;
        dialog.exec();
        newSample(cloudToProcess, res);
        break;
    }
    case 1:
    {
        //QMessageBox::information(this, "Information", u8"��ʼRandom����ȴ��㷨��ɡ�����");
        randomInputDialog dialog;
        dialog.exec();
        randomSample(cloudToProcess, res);
        break;
    }
    case 2:
    {
        //QMessageBox::information(this, "Information", u8"��ʼvoxel������");
        voxelInputDialog dialog;
        dialog.exec();
        voxelSample(cloudToProcess, res, 0.5);
        break;
    }
    case 3:
    {
        //QMessageBox::information(this, "Information", u8"��ʼcurvature������");
        curvatureInputDialog dialog;
        dialog.exec();
        randomSample(cloudToProcess, res);
        break;
    }
    case 4:
        QMessageBox::information(this, "Information", u8"meshing������");
        break;
    default:
        QMessageBox::information(this, "Information", u8"err������");
        break;
    }
    QMessageBox::information(this, "Information", u8"������ɣ�");
    
    if (editFlag) {
        cloudTar = res;
    }
    else {
        cloudSrc = res;
    }

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

void realTimeProcess::editSrc() {
    editFlag = 0;
}

void realTimeProcess::editTar() {
    editFlag = 1;
}

void realTimeProcess::mergeCloud() {
    if (!isLoadSrc) {
        errorInfo(2);
        return;
    }
    if (!isLoadTar) {
        errorInfo(3);
        return;
    }
    *cloudTar+=*cloudSrc;
    isLoadSrc = 0;
    updatePointCloudShow();
}

void realTimeProcess::stopSystem() {
    isRunning.store(false, memory_order_release);
}

void realTimeProcess::realTimeSys() {
    //ѡ���ļ���
    //��ȡ�ļ�·��
    path = QFileDialog::getExistingDirectory(this, "ѡ�����ļ���");
    {
        unique_lock<mutex> textLck(textMutex);
        ui.textBrowser->append(u8"����ļ��У�" + path + "\n");
    }
    logger.log("����ļ��У�" + path.toStdString() + "\n");
    if (path.isEmpty()) {
        unique_lock<mutex> textLck(textMutex);
        ui.textBrowser->append(u8"����ļ���Ϊ�գ�������ѡ��\n");
    }
    logger.log("�����߳�ǰ\n");
    //QThread* m_workerThread = new QThread;
    //connect(m_workerThread, &QThread::started, this, &realTimeProcess::folderMonitor);
    //m_workerThread->start();
    //m_workerThread->wait();
    //QMessageBox::information(this, "Information", u8"�������С�����");
    std::thread worker(&realTimeProcess::folderMonitor, this);
    worker.detach();
}

void realTimeProcess::showDialog() {
    QDialog* m_progressDialog = new QDialog(qobject_cast<QWidget*>(parent()));
    QLabel* label = new QLabel(u8"���ڽ���ʵʱƴ��...", m_progressDialog);
    QPushButton* abortButton = new QPushButton(u8"��ֹ", m_progressDialog);

    QVBoxLayout* layout = new QVBoxLayout(m_progressDialog);
    layout->addWidget(label);
    layout->addWidget(abortButton);
    m_progressDialog->setLayout(layout);

    // ������ֹ�ź�
    connect(abortButton, &QPushButton::clicked,
        this, &realTimeProcess::stopSystem);

    // ��ʾ��ģ̬�Ի���
    m_progressDialog->show();
    // ȷ���Ի���ɼ�
    m_progressDialog->show();
    m_progressDialog->raise();  // �ö���ʾ
    m_progressDialog->activateWindow(); // �����
    m_progressDialog->resize(300, 100); // ��ȷ���ó�ʼ�ߴ�
}

void realTimeProcess::updateWhole() {
    view->removePointCloud("cloudShow");
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloudShow);
    view->addPointCloud(cloudShow, rgb, "cloudShow");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudShow");
    view->resetCamera();
    view->spin();
    ui.openGLWidget->setRenderWindow(view->getRenderWindow());
}

//���ĵ�ǰ״̬Ϊ������-��ȡ�ļ�·��-��ѯ�ļ�·��
void realTimeProcess::folderMonitor()
{
    logger.log("�������̣߳���ʼ����״̬\n");
    //����״̬
    isRunning.store(true, std::memory_order_seq_cst);
    logger.log("״̬������ɣ���ʼ�����ļ�\n");
    QDir dir(path);
    QSet<QString> knownFiles;
    while (isRunning.load(memory_order_acquire)) {
        QThread::msleep(1000);
        QCoreApplication::processEvents();
        QSet<QString> currentFiles = dir.entryList({ "*.ply", "*.pcd" }, QDir::Files).toSet();
        auto newFiles=currentFiles - knownFiles;
        for (const QString& file : newFiles) {
            if (!isRunning.load(memory_order_acquire))break;
            knownFiles.insert(file);
            logger.log("��ʼ����"+file.toStdString()+"\n");
            QString fullPath = dir.filePath(file);
            PointCloudT::Ptr cloud(new PointCloudT);

            bool success = false;
            if (fullPath.endsWith(".ply")) {
                (pcl::io::loadPLYFile(fullPath.toStdString(), *cloud) == 0);
            }
            else if (fullPath.endsWith(".pcd")) {
                (pcl::io::loadPCDFile(fullPath.toStdString(), *cloud) == 0);
            }
            //��ȡ�ɹ���ʼ��׼
            if (cloud->size() < 100) {
                logger.log("��ȡʧ��\n");
                continue;
            }
            logger.log("��ȡ�ɹ�\n");
            //��ʼ����
            if (cloudNum == 0) {
                points.push_back(cloud);
                cloudNum++;
                PointCloudT::Ptr down(new PointCloudT());;
                voxelSample(cloud, down, 0.1);
                matcher.addNewFeature(down);
                pointsDown.push_back(down);
                *cloudShowDown+=*down;
                *cloudShow+=*cloud;
                logger.log("��һ֡���ƣ�ֱ�����\n");
            }
            else {
                //����ƥ��
                PointCloudT::Ptr down(new PointCloudT());;
                voxelSample(cloud, down, 0.1);
                int index=matcher.match(down);
                {
                    unique_lock<mutex> textLck(textMutex);
                    ui.textBrowser->append(u8"��ƥ����ƣ�" + QString::number(index) + "\n");
                }
                bool regSuccess = false;
                Eigen::Matrix4f trans=gicpReg(down, pointsDown[index], regSuccess);
                if (!regSuccess) {
                    {
                        unique_lock<mutex> textLck(textMutex);
                        ui.textBrowser->append(u8"�ֲ�ƴ�Ӵ��󣬿�ʼȫ��ƴ��\n");
                    }
                    trans = gicpReg(down, cloudShowDown, regSuccess);
                }
                if (!regSuccess) {
                    {
                        unique_lock<mutex> textLck(textMutex);
                        ui.textBrowser->append(u8"ƴ��ʧ��\n");
                    }
                    continue;
                }
                pcl::transformPointCloud(*cloud, *cloud, trans);
                matcher.addNewFeature(down);
                points.push_back(cloud);
                cloudNum++;
                pointsDown.push_back(down);
                *cloudShowDown+=*down;
                *cloudShow+=*cloud;
            }
            logger.log("cloudShow:"+to_string(cloudShow->size())+"\n");
            logger.log("ƴ�ӳɹ�����ʼ���ӻ�\n");

            QMetaObject::invokeMethod(this, "updateWhole", Qt::QueuedConnection);
            logger.log("���ӻ��������\n");
            QThread::msleep(1000);
        }
    }
    
    QMetaObject::invokeMethod(this, "PostProcessWhole", Qt::QueuedConnection);
}

void realTimeProcess::PostProcessWhole() {
    QMessageBox::information(this, "Information", u8"��ʼ������ʱ������ȴ�������");
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloudShow);
    ne.setInputCloud(cloudShow);
    ne.setSearchMethod(tree);
    ne.setKSearch(15);  // �������[1,5](@ref)
    ne.compute(*normals);

    // Step 2: �ϲ���ɫ�뷨����Ϣ
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloudShow, *normals, *cloud_with_normals);

    // Step 3: �����ؽ�������RGB��ɫ��
    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    poisson.setInputCloud(cloud_with_normals);
    poisson.setDepth(9);          
    poisson.setSolverDivide(8);  
    poisson.reconstruct(*mesh);

    // Step 4: ���¿��ӻ����
    view->removeAllPointClouds();
    view->removeAllShapes();
    view->addPolygonMesh(*mesh, "reconstructed_mesh");
    view->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "reconstructed_mesh"); // ��͸����ʾ[5](@ref)
    view->spinOnce(10); // ������ˢ��[3](@ref)
}