#include "realTimeProcess.h"

realTimeProcess::realTimeProcess(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);
    //初始化相关点云
    cloudShow.reset(new PointCloudT());
    cloudTar.reset(new PointCloudT());
    cloudSrc.reset(new PointCloudT());
    cloudShowDown.reset(new PointCloudT());
    initialVtkWidget();

    //输入相关
    connect(ui.actionloadTarFile, SIGNAL(triggered()), this, SLOT(loadTar()));
    connect(ui.actionloadSrcFile, SIGNAL(triggered()), this, SLOT(loadSrc()));
    connect(ui.loadSrcCloud, SIGNAL(clicked()), this, SLOT(loadSrc()));
    connect(ui.loadTarCloud, SIGNAL(clicked()), this, SLOT(loadTar()));

    //当前编辑的点云
    connect(ui.editSrcCloud, SIGNAL(clicked()), this, SLOT(editSrcCloud()));
    connect(ui.editTarCloud, SIGNAL(clicked()), this, SLOT(editTarCloud()));

    //合并点云
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
    //QMessageBox::information(this, "Information", u8"进入系统···");
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
    case 0: {
            gicpInputDialog dialog;
            dialog.exec();
            //QString infos = QString("%1,%2,%3,%4").arg(dialog.getMaxDistance()).arg(dialog.getScale()).arg(dialog.getMaxDistance()).arg(dialog.getReciprocalCorrespondences());
            //QMessageBox::information(this, "Information", infos);
            transform = gicpReg(cloudSrc, cloudTar, flag);
            break;
        }
    case 1: {
            //QMessageBox::information(this, "Information", u8"开始改进GICP，请等待算法完成···");
            myGicpInputDialog dialog;
            dialog.exec();
            transform = multi_scaling_gicp(cloudSrc, cloudTar, flag);
            break;
        }
            
    case 2: {
            //QMessageBox::information(this, "Information", u8"开始ICP配准···");
            icpInputDialog dialog;
            dialog.exec();
            transform = icpReg(cloudSrc, cloudTar, flag);
            break;
        }
            
    case 3: {
            //QMessageBox::information(this, "Information", u8"开始NICP配准···");
            nicpInputDialog dialog;
            dialog.exec();
            transform = normalIcpReg(cloudSrc, cloudTar, flag);
            break;
        }
            
    case 4: {
        //QMessageBox::information(this, "Information", u8"开始nonlinearICP配准···");
        nonlinearIcpInputDialog dialog;
        dialog.exec();
        transform = nlIcpReg(cloudSrc, cloudTar, flag);
        break;
        }
            
    case 5: {
        //QMessageBox::information(this, "Information", u8"开始FPFH-SAC配准···");
        fpfhInputDialog dialog;
        dialog.exec();
        transform = fpfhReg(cloudSrc, cloudTar);
        break;
        }
            
    case 6: {
        //QMessageBox::information(this, "Information", u8"开始4pcs配准···");
        fpcsInputDialog dialog;
        dialog.exec();
        transform = FpcsReg(cloudSrc, cloudTar);
        break;
        }
            
    case 7: {
        //QMessageBox::information(this, "Information", u8"开始k4pcs配准···");
        kfpcsInputDialog dialog;
        dialog.exec();
        transform = kfpcs(cloudSrc, cloudTar);
        break;
        }
            
    case 8: {
        //QMessageBox::information(this, "Information", u8"开始NDT配准···");
        NDTInputDialog dialog;
        dialog.exec();
        transform = NDT(cloudSrc, cloudTar);
        break;
        }
            
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
    //参数校验
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
    {
        //QMessageBox::information(this, "Information", u8"开始ISS，请等待算法完成···");
        issInputDialog dialog;
        dialog.exec();
        newSample(cloudToProcess, res);
        break;
    }
    case 1:
    {
        //QMessageBox::information(this, "Information", u8"开始Random，请等待算法完成···");
        randomInputDialog dialog;
        dialog.exec();
        randomSample(cloudToProcess, res);
        break;
    }
    case 2:
    {
        //QMessageBox::information(this, "Information", u8"开始voxel···");
        voxelInputDialog dialog;
        dialog.exec();
        voxelSample(cloudToProcess, res, 0.5);
        break;
    }
    case 3:
    {
        //QMessageBox::information(this, "Information", u8"开始curvature···");
        curvatureInputDialog dialog;
        dialog.exec();
        randomSample(cloudToProcess, res);
        break;
    }
    case 4:
        QMessageBox::information(this, "Information", u8"meshing···");
        break;
    default:
        QMessageBox::information(this, "Information", u8"err···");
        break;
    }
    QMessageBox::information(this, "Information", u8"计算完成！");
    
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
    //选择文件夹
    //读取文件路径
    path = QFileDialog::getExistingDirectory(this, "选择监控文件夹");
    {
        unique_lock<mutex> textLck(textMutex);
        ui.textBrowser->append(u8"监控文件夹：" + path + "\n");
    }
    logger.log("监控文件夹：" + path.toStdString() + "\n");
    if (path.isEmpty()) {
        unique_lock<mutex> textLck(textMutex);
        ui.textBrowser->append(u8"监控文件夹为空，请重新选择！\n");
    }
    logger.log("启动线程前\n");
    //QThread* m_workerThread = new QThread;
    //connect(m_workerThread, &QThread::started, this, &realTimeProcess::folderMonitor);
    //m_workerThread->start();
    //m_workerThread->wait();
    //QMessageBox::information(this, "Information", u8"降采样中···");
    std::thread worker(&realTimeProcess::folderMonitor, this);
    worker.detach();
}

void realTimeProcess::showDialog() {
    QDialog* m_progressDialog = new QDialog(qobject_cast<QWidget*>(parent()));
    QLabel* label = new QLabel(u8"正在进行实时拼接...", m_progressDialog);
    QPushButton* abortButton = new QPushButton(u8"终止", m_progressDialog);

    QVBoxLayout* layout = new QVBoxLayout(m_progressDialog);
    layout->addWidget(label);
    layout->addWidget(abortButton);
    m_progressDialog->setLayout(layout);

    // 连接终止信号
    connect(abortButton, &QPushButton::clicked,
        this, &realTimeProcess::stopSystem);

    // 显示非模态对话框
    m_progressDialog->show();
    // 确保对话框可见
    m_progressDialog->show();
    m_progressDialog->raise();  // 置顶显示
    m_progressDialog->activateWindow(); // 激活窗口
    m_progressDialog->resize(300, 100); // 明确设置初始尺寸
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

//更改当前状态为运行中-读取文件路径-轮询文件路径
void realTimeProcess::folderMonitor()
{
    logger.log("进入子线程，开始更改状态\n");
    //更新状态
    isRunning.store(true, std::memory_order_seq_cst);
    logger.log("状态更改完成，开始遍历文件\n");
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
            logger.log("开始处理"+file.toStdString()+"\n");
            QString fullPath = dir.filePath(file);
            PointCloudT::Ptr cloud(new PointCloudT);

            bool success = false;
            if (fullPath.endsWith(".ply")) {
                (pcl::io::loadPLYFile(fullPath.toStdString(), *cloud) == 0);
            }
            else if (fullPath.endsWith(".pcd")) {
                (pcl::io::loadPCDFile(fullPath.toStdString(), *cloud) == 0);
            }
            //读取成功后开始配准
            if (cloud->size() < 100) {
                logger.log("读取失败\n");
                continue;
            }
            logger.log("读取成功\n");
            //初始点云
            if (cloudNum == 0) {
                points.push_back(cloud);
                cloudNum++;
                PointCloudT::Ptr down(new PointCloudT());;
                voxelSample(cloud, down, 0.1);
                matcher.addNewFeature(down);
                pointsDown.push_back(down);
                *cloudShowDown+=*down;
                *cloudShow+=*cloud;
                logger.log("第一帧点云，直接添加\n");
            }
            else {
                //进行匹配
                PointCloudT::Ptr down(new PointCloudT());;
                voxelSample(cloud, down, 0.1);
                int index=matcher.match(down);
                {
                    unique_lock<mutex> textLck(textMutex);
                    ui.textBrowser->append(u8"最匹配点云：" + QString::number(index) + "\n");
                }
                bool regSuccess = false;
                Eigen::Matrix4f trans=gicpReg(down, pointsDown[index], regSuccess);
                if (!regSuccess) {
                    {
                        unique_lock<mutex> textLck(textMutex);
                        ui.textBrowser->append(u8"局部拼接错误，开始全局拼接\n");
                    }
                    trans = gicpReg(down, cloudShowDown, regSuccess);
                }
                if (!regSuccess) {
                    {
                        unique_lock<mutex> textLck(textMutex);
                        ui.textBrowser->append(u8"拼接失败\n");
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
            logger.log("拼接成功，开始可视化\n");

            QMetaObject::invokeMethod(this, "updateWhole", Qt::QueuedConnection);
            logger.log("可视化更新完成\n");
            QThread::msleep(1000);
        }
    }
    
    QMetaObject::invokeMethod(this, "PostProcessWhole", Qt::QueuedConnection);
}

void realTimeProcess::PostProcessWhole() {
    QMessageBox::information(this, "Information", u8"开始后处理，耗时长，请等待···");
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloudShow);
    ne.setInputCloud(cloudShow);
    ne.setSearchMethod(tree);
    ne.setKSearch(15);  // 邻域点数[1,5](@ref)
    ne.compute(*normals);

    // Step 2: 合并颜色与法线信息
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloudShow, *normals, *cloud_with_normals);

    // Step 3: 泊松重建（适配RGB颜色）
    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    poisson.setInputCloud(cloud_with_normals);
    poisson.setDepth(9);          
    poisson.setSolverDivide(8);  
    poisson.reconstruct(*mesh);

    // Step 4: 更新可视化组件
    view->removeAllPointClouds();
    view->removeAllShapes();
    view->addPolygonMesh(*mesh, "reconstructed_mesh");
    view->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "reconstructed_mesh"); // 半透明显示[5](@ref)
    view->spinOnce(10); // 非阻塞刷新[3](@ref)
}