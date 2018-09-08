mpIniORBextractor features is two size of mpORBextractorLeft

in Frame():
    ExtractORB(0, imGray);
    =>ORBextractor::operator()
      =>ComputePyramid(image);
      =>ComputeKeyPointsOctTree(allKeypoits);
            in each level, pick points in EDGE_THRESHOLD
            whith W=30, make cell in wCell x hCell
              =>in each cell, FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                对于每一层, keypoints = DistributeOctTree()                                 vKeysCell,iniThFAST,true);
                  =>对每一个node，记录四个坐标点，将vpIniNodes指向lNodes，将每个待分配的keyPoint分配到lNodes中，并检查是否每个node上只有一个kp，若为1个置bNoMore为true
                    If more than one point, subdivide
                    Add childs if they contain points
                    Finish if there are more nodes than required features
                            // or all nodes contain just one point
                    Retain the best point in each node
                    


turn to Track();
...
turn to mState==OK
    CheckReplacedInLastFrame()
       用local map产生的mpReplaced替代原有的mvpMapPoint
    有速度模型，当速度为空或者当前mnId与mnLastRelocFrameId相差不大时
        TrackReferenceKeyFrame
            先计算Bag of Words vector，mCurrentFrame.ComputeBoW();
                如果词袋为空，则拆分mDescriptors，再转化到mBowVec,mFeatVec
            与参考的关键帧进行ORB匹配，如果有足够的匹配点，则进行PnP求解
                构造一个ORBmatcher matcher(0.7,true)
                matcher.SearchByBoW => nmatches
                    搜索中，将同一个level的相同词汇 node作为匹配
                        通过pKF->mFeatVec继承的map键值作为id，找到对应的地图点
                        判断地图点的可用性，再找到对应的描述子
                            再对当前帧的序列做遍历，对每个空的vpMapPointMatches[realIdxF]
            PoseOptimization，采用g2o进行相机位姿优化
                必须要有3个以上的优化边
                进行4次优化求解：在每次求解(10循环)后，将观测点划分为内点和外点，在下一次优化里不包含外点，但是外点可以被重新划分为内点
                    在每次求解中，将最大的误差点作为每次循环的误差，在第二次后取消RobustKernel；每次优化都是从同一个初始状态开始pFrame->mTcw();
                最后将优化的Tcw存于pFrame中，并返回坏点之外的优化所用点
            对每个mvbOutlier（在PoseOptimization中设定）
                清除mvpMapPoints中保存的指针
                取消mbTrackView标志
            如果匹配点大于10则返回true
        TrackLocalMap
            UpdateLocalMap
                可视化设置
                UpdateLocalKeyFrames
                    对当前帧的每个地图点进行统计观测到的关键帧
                    清除mvpLocalKeyFrames
                    将每个观测到一个地图点的关键帧都存到mvpLocalKeyFrames中，并统计最大观测关键帧
                    把关键帧的mnTrackReferenceForFrame设为当前的帧
                    GetBestCovisibilityKeyFrames把关键帧的相邻帧也加到清除mvpLocalKeyFrames
                        锁住mMutexConnections
                        返回关键帧中维护的mvpOrderedConnectedKeyFrames
                    只添加最近的未加入帧
                    获取关键帧的GetChilds
                    加入第一个未加入的child
                    获取关键帧的GetParent
                    加入第一个未加入的cParent
                    将Tracker中的mpReferenceKF设置为观测量最大的关键帧，同时将当前帧的参考帧也设这个
                UpdateLocalPoints
                    清除mvpLocalMapPoints
                    对刚刚得到的清除mvpLocalKeyFrames，更新每个MapPoint的mnTrackReferenceForFrame和重构mvpLocalMapPoints
                SearchLocalPoints
                    不搜索那些已经匹配上的map points
                        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                        pMP->mbTrackInView = false;
                    Project points in frame and check its visibility
                Optimize Pose
                // Decide if the tracking was succesful
                // More restrictive if there was a relocalization recently
        如果成功则将mState状态更新为OK，否则更新为LOST
        更新drawer，mpFrameDrawer->Update()
        If tracking were good, check if we insert a keyframe
            Update motion model，mVelocity设置为上一帧到这个帧的转移矩阵相乘
        Clean VO matches
        Delete temporal MapPoints
        Check if we need to insert a new keyframe
            If Local Mapping is freezed by a Loop Closure do not insert keyframes
            Do not insert keyframes if not enough frames have passed from last relocalisation


        LocalMapping::ProcessNewKeyFrame
            ...部分内容在台式机
            更新当前关键帧的链接
                mpCurrentKeyFrame->UpdateConnections()
                根据MapPoint的观测关键帧，将链接信息更新到mConnectedKeyFrameWeights、mvpOrderedConnectedKeyFrames、mvOrderedWeights
                如果是第一次更新，则将mpParent设置为最多的观测帧，并将本关键帧设为child
            将Keyframe插入到Map

        LocalMapping::MapPointCulling，检查最近的MapPoints
            最近添加的地图点在mlpRecentAddedMapPoints
            nThObs = 2，cnThObs=2
            如果点是bad状态，删除这个点
            (mnFound)/mnVisible的比例在0.25以内，SetBadFlag，删除这个点
                锁住mMutexFeatures和mMutexPos，mbBad=true，删除mObservations，对每个点对应的KeyFrame都删除这个点，对mpMap删除这个点
            如果在两次关键帧id更新后，观测数量在2以内就删除这个点
            如果在三次关键帧id更新后，就将这个点从mlpRecentAddedMapPonts中移除

        LocalMapping::CreateNewMapPoints，三角化新的地图点
            在Covisibility图中检索邻居关键帧
            nn = 2，vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn)
                锁住mMutexConnections
                从mvpOrderedConnectedKeyFrames返回指定数量的相邻帧
            建立新的ORBmatcher matcher(0.6, false)
            获取当前关键帧的Tcw1
            Search matches with epipolar restriction and triangulate
                对每个vpNeighKFs的关键帧
                第一帧除外，如果有新的关键帧，就先返回
                Check first that baseline is not too short
                    计算帧与帧之间的基线长度
                计算pKF2的场景中位深度，用baseline/medianDepthKF2，小于0.01则放弃
                计算基础矩阵F12
                matcher.SearchForTriangulation

        mnCovisibilityConsistencyTh = 3
        LoopClosing::DetectLoop()
            ..
            获取mpCurrentKF->mBowVec，进行得分的计算
                判断链接中pKF的状态，然后提取它的BowVec进行得分的计算（DBow）。
                获取所有链接的pKF的最低分
            传入最低分，mpKeyFrameDB->DetectLoopCandidates()
                GetConnectedKeyFrames，从临域的mCnnectedKeyFrameweights中抽取
                锁定mMutex
                    获取mvInvertedFile，对每一个相关的关键帧‘
                        如果CnnectedKeyFrameweights没有含有当前帧，开始计数，推入lKFsSharingWords
                    lKFsSharingWords为空，则返回一个空的容器
                获取最大的共享词典的数量，minCommonWords = maxCommonWords*0.8f
                构建得分相关的lScoreAndMatch
                    如果都没有>=minScore的关键帧，返回空的容器
                对lScoreAndMatch
                    每一个关键帧，获取其10个临近关键帧
                        在临近关键帧中，检查mnLoopQuery是否是当前关键帧，mnLoopWords是否大于minCommonWords
                            若是，再考虑mLoopScore是否大于bestScore，是则更新pBestKF
                        构建lAccScoreAndMatch
                minScoreToRetain = 0.75f*bestAccScore
                    用minScoreToRetain来保留所有的关键帧，去重
            If there are no loop candidates, just add new keyframe and return false
            // For each loop candidate check consistency with previous loop candidates
            // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
            // A group is consistent with a previous group if they share at least a keyframe
            // We must detect a consistent loop in several consecutive keyframes to accept it
            mvpEnoughConsistentCandidates.clear()
            对得到的结果vpCandidateKFs,每个关键帧获取它的GetConnectedKeyFrames
                spCandidateGroup中插入选定的关键帧
                对mvConsistentGroups轮巡
                    对sPreviousGroup轮巡
                        mvConsistentGroups是之前循环的数据，看是否包含了这次轮巡的KeyFrame，有包含的bConsistent=bConsistentForSomeGroup=true，break
                    如果有连续信号
                        更新nCurrentConsistency
                        vCurrentConsistentGroups.push_back(cg)
                        vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                        如果有足够的链接帧，mvpEnoughConsistentCandidates.push_back(pCandidateKF)，bEnoughConsistent=true
                    如果一个连续信号都没，vCurrentConsistentGroups.push_back(spCandidateGroup,0)
            更新mvConsistentGroups = vCurrentConsistentGroups
            Add Current Keyframe to database，mpKeyFrameDB->add(mpCurrentKF)
            判断mvpEnoughConsistentCandidates，如有则返回true，否则为false

        LoopClosing::ComputeSim3()
            For each consistent loop candidate we try to compute a Sim3
            构建临时的求解器vector<Sim3Solver*> vpSim3Solvers
            ...
            进入Ransac的主循环
                Sim3Solver::ComputeSim3
                    依照论文的方法，构建了旋转mR12i、尺度ms12i、平移mT12i、转换矩阵mT12i/mT21i
                Sim3Solver::CheckInliers()
                    通过转换矩阵，交叉将地图点投影到vP1im图像点上
                    与初始化时投影的点比较，得到误差值，需要误差值在范围内，才算为内点
                如果内点的数量大于最佳估计，就将这个估计作为输出
                    返回的是mBestT12，bNoMore为false，要是都没达到最低要求，就是true
                若bNoMore，忽略当前帧
                若有返回Translation
                    在matcher中SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5)
                    获取两个关键帧的转移矩阵
                    设置vbAlreadyMatched标志
                    对KF1中的每个地图点
                        将它们分别投影到两个位姿的坐标系下
                        计算KF2的图像坐标、图层
                            搜索radius内的Features
                            从描述子的角度看相似度
                            若最佳相似度小于TH，标记vnMatch
                    对KF2中的每个地图点，也记录vnMatch2
                    若两者能对应上，则增加nFound，记录点对到vpMatches12
                建立g2o::Sim3表示，进行Optimizer::OptimizeSim3
                    只更新sim3，进行两次优化，第二次去除外点
                返回的nInliers大于20个，设置bMatch、mpmatchedKF
                更新mScw和mvpCurrentMatchedPoints
            若没有匹配上bMatch
                清楚mvpEnoughConsistentCandidates
                设置当前关键帧SetErase
            获取第二个关键真看到的地图点
                添加到mvpLoopMapPoints，设置mnLoopPointForKF的id号
            进行SearchByProjection，获取更多的点，用的是计算得到的Sim3
            得到mvpCurrentMatchedPoints的数量大于40个，设置SetRrase不是mpMatchedKF的关联关键帧
                否则全设

        LoopClosing::CorrectLoop()