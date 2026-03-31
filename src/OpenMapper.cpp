/*
 * Copyright (C) 2006-2011, SRI International (R)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <thread>
#include <vector>

#include <stdexcept>
#include <queue>
#include <set>
#include <iterator>
#include <map>
#include <iostream>

#include <OpenMapper.h>
#include <Logger.h>

namespace karto
{

  // enable this for verbose debug information
  //#define KARTO_DEBUG
  //#define KARTO_DEBUG2

  #define MAX_VARIANCE            500.0
  #define DISTANCE_PENALTY_GAIN   0.2
  #define ANGLE_PENALTY_GAIN      0.2

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Manages the data for a sensor
   */
  class SensorDataManager
  {
  public:
    /**
     * Default constructor
     */
    SensorDataManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
      : m_pLastScan(nullptr)
      , m_RunningBufferMaximumSize(runningBufferMaximumSize)
      , m_RunningBufferMaximumDistance(runningBufferMaximumDistance)
    {
    }

    /**
     * Destructor
     */
    virtual ~SensorDataManager()
    {
      Clear();
    }

  public:
    /**
     * Adds objects to list of objects, tagging object with given unique id;
     * if object is a scan, then scan gets added to list of processed scans
     * @param pObject object
     * @param uniqueId unique id
     */
    inline void AddObject(LocalizedObject* pObject, kt_int32s uniqueId)
    {
      // assign state id to object
      pObject->SetStateId(static_cast<kt_int32u>(m_Objects.size()));

      // assign unique id to object
      pObject->SetUniqueId(uniqueId);

      m_Objects.push_back(pObject);

      // if object is scan and it was scan-matched, add it to scan buffer
      LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
      if (pScan != nullptr)
      {
        m_Scans.push_back(pScan);
      }      
    }

    /**
     * Gets last scan
     * @return last localized scan
     */
    inline LocalizedLaserScan* GetLastScan()
    {
      return m_pLastScan;
    }

    /**
     * Sets the last scan
     * @param pScan
     */
    inline void SetLastScan(LocalizedLaserScan* pScan)
    {
      m_pLastScan = pScan;
    }

    /**
     * Gets objects
     * @return objects
     */
    inline LocalizedObjectList& GetObjects()
    {
      return m_Objects;
    }
    
    /**
     * Gets scans
     * @return scans
     */
    inline LocalizedLaserScanList& GetScans()
    {
      return m_Scans;
    }
    
    /**
     * Gets index of scan in sensor's list of scans
     * @param pScan
     * @return index into scans list; -1 if not found
     */
    inline kt_int32s GetScanIndex(LocalizedLaserScan* pScan)
    {
      kt_int32s targetStateId = pScan->GetStateId();
      auto it = std::lower_bound(m_Scans.begin(), m_Scans.end(), targetStateId,
        [](LocalizedLaserScan* a, kt_int32s stateId) {
          return a->GetStateId() < stateId;
        });
      if (it != m_Scans.end() && (*it)->GetStateId() == targetStateId)
      {
        return static_cast<kt_int32s>(std::distance(m_Scans.begin(), it));
      }
      return -1;
    }

    /**
     * Gets running scans
     * @return running scans
     */
    inline LocalizedLaserScanList& GetRunningScans()
    {
      return m_RunningScans;
    }

    /**
     * Adds scan to list of running scans
     * @param pScan
     */
    void AddRunningScan(LocalizedLaserScan* pScan)
    {
      m_RunningScans.push_back(pScan);

      // list has at least one element (first line of this function), so this is valid
      Pose2 frontScanPose = m_RunningScans.front()->GetSensorPose();
      Pose2 backScanPose = m_RunningScans.back()->GetSensorPose();

      // cap list size and remove all scans from front of list that are too far from end of list
      kt_double squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
      while (m_RunningScans.size() > m_RunningBufferMaximumSize || squaredDistance > math::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE)
      {
        // remove front of running scans
        m_RunningScans.erase(m_RunningScans.begin());

        // recompute stats of running scans
        frontScanPose = m_RunningScans.front()->GetSensorPose();
        backScanPose = m_RunningScans.back()->GetSensorPose();
        squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
      }
    }

    /**
     * Deletes data of this buffered sensor
     */
    void Clear()
    {
      m_Objects.clear();
      m_Scans.clear();
      m_RunningScans.clear();
      m_pLastScan = nullptr;
    }

  private:
    static kt_int32s ScanIndexComparator(LocalizedLaserScan* pScan1, LocalizedLaserScan* pScan2)
    {
      return pScan1->GetStateId() - pScan2->GetStateId();
    }
    
  private:
    LocalizedObjectList m_Objects;
    
    LocalizedLaserScanList m_Scans;
    LocalizedLaserScanList m_RunningScans;
    LocalizedLaserScan* m_pLastScan;

    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;
  }; // SensorDataManager

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  using SensorDataManagerMap = std::map<Identifier, SensorDataManager*>;

  struct MapperSensorManagerPrivate
  {
    // map from sensor name to sensor data
    SensorDataManagerMap m_SensorDataManagers;
    
    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;
    
    kt_int32s m_NextUniqueId;    
    
    LocalizedObjectList m_Objects;
  };

  MapperSensorManager::MapperSensorManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
    : m_pMapperSensorManagerPrivate(new MapperSensorManagerPrivate())
  {
    m_pMapperSensorManagerPrivate->m_RunningBufferMaximumSize = runningBufferMaximumSize;
    m_pMapperSensorManagerPrivate->m_RunningBufferMaximumDistance = runningBufferMaximumDistance;
    m_pMapperSensorManagerPrivate->m_NextUniqueId = 0;
  }
  
  MapperSensorManager::~MapperSensorManager()
  {
    Clear();
    delete m_pMapperSensorManagerPrivate;
  }
  
  
  void MapperSensorManager::RegisterSensor(const Identifier& rSensorName)
  {
    if (GetSensorDataManager(rSensorName) == nullptr)
    {
      m_pMapperSensorManagerPrivate->m_SensorDataManagers[rSensorName] = new SensorDataManager(m_pMapperSensorManagerPrivate->m_RunningBufferMaximumSize, m_pMapperSensorManagerPrivate->m_RunningBufferMaximumDistance);
    }
  }

  LocalizedObject* MapperSensorManager::GetLocalizedObject(const Identifier& rSensorName, kt_int32s stateId)
  {
    SensorDataManager* pSensorDataManager = GetSensorDataManager(rSensorName);
    if (pSensorDataManager != nullptr)
    {
      return pSensorDataManager->GetObjects()[stateId];
    }

    assert(false);
    return nullptr;
  }
  
  // for use by scan solver
  LocalizedObject* MapperSensorManager::GetLocalizedObject(kt_int32s uniqueId)
  {
    assert(math::IsUpTo(uniqueId, (kt_int32s)m_pMapperSensorManagerPrivate->m_Objects.size()));
    return m_pMapperSensorManagerPrivate->m_Objects[uniqueId];
  }
  
  std::vector<Identifier> MapperSensorManager::GetSensorNames()
  {
    std::vector<Identifier> sensorNames;
    for (auto& entry : m_pMapperSensorManagerPrivate->m_SensorDataManagers)
    {
      sensorNames.push_back(entry.first);
    }

    return sensorNames;
  }
  
  LocalizedLaserScan* MapperSensorManager::GetLastScan(const Identifier& rSensorName)
  {
    return GetSensorDataManager(rSensorName)->GetLastScan();
  }
  
  void MapperSensorManager::SetLastScan(LocalizedLaserScan* pScan)
  {
    GetSensorDataManager(pScan)->SetLastScan(pScan);
  }
  
  void MapperSensorManager::ClearLastScan(const Identifier& rSensorName)
  {
    GetSensorDataManager(rSensorName)->SetLastScan(nullptr);
  }

  void MapperSensorManager::AddLocalizedObject(LocalizedObject* pObject)
  {
    GetSensorDataManager(pObject)->AddObject(pObject, m_pMapperSensorManagerPrivate->m_NextUniqueId);
    m_pMapperSensorManagerPrivate->m_Objects.push_back(pObject);
    m_pMapperSensorManagerPrivate->m_NextUniqueId++;
  }
  
  void MapperSensorManager::AddRunningScan(LocalizedLaserScan* pScan)
  {
    GetSensorDataManager(pScan)->AddRunningScan(pScan);
  }
  
  LocalizedLaserScanList& MapperSensorManager::GetScans(const Identifier& rSensorName)
  {
    return GetSensorDataManager(rSensorName)->GetScans();
  }
  
  kt_int32s MapperSensorManager::GetScanIndex(LocalizedLaserScan* pScan)
  {
    return GetSensorDataManager(pScan->GetSensorIdentifier())->GetScanIndex(pScan);
  }

  LocalizedLaserScanList& MapperSensorManager::GetRunningScans(const Identifier& rSensorName)
  {
    return GetSensorDataManager(rSensorName)->GetRunningScans();
  }
  
  LocalizedLaserScanList MapperSensorManager::GetAllScans()
  {
    LocalizedLaserScanList scans;

    for (auto& entry : m_pMapperSensorManagerPrivate->m_SensorDataManagers)
    {
      LocalizedLaserScanList& rScans = entry.second->GetScans();
      scans.insert(scans.end(), rScans.begin(), rScans.end());
    }

    return scans;
  }

  karto::LocalizedObjectList MapperSensorManager::GetAllObjects()
  {
    LocalizedObjectList objects;

    for (auto& entry : m_pMapperSensorManagerPrivate->m_SensorDataManagers)
    {
      LocalizedObjectList& rObjects = entry.second->GetObjects();
      objects.insert(objects.end(), rObjects.begin(), rObjects.end());
    }

    return objects;
  }

  void MapperSensorManager::Clear()
  {
    for (auto& entry : m_pMapperSensorManagerPrivate->m_SensorDataManagers)
    {
      delete entry.second;
    }
    
    m_pMapperSensorManagerPrivate->m_SensorDataManagers.clear();
  }
  
  SensorDataManager* MapperSensorManager::GetSensorDataManager(const Identifier& rSensorName)
  {
    if (m_pMapperSensorManagerPrivate->m_SensorDataManagers.find(rSensorName) != m_pMapperSensorManagerPrivate->m_SensorDataManagers.end())
    {
      return m_pMapperSensorManagerPrivate->m_SensorDataManagers[rSensorName];
    }
    
    return nullptr;
  }  

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
   
  class ScanMatcherGridSetBank
  {
  public:
    ScanMatcherGridSetBank(kt_int32u nGrids, kt_int32s corrGridWidth, kt_int32s corrGridHeight,
                           kt_double resolution, kt_double smearDeviation,
                           kt_int32s searchSpaceGridWidth, kt_int32s searchSpaceGridHeight)
    {
      if (nGrids <= 0)
      {
        throw Exception("ScanMatcherGridSetBank requires at least 1 grid: " + StringHelper::ToString(nGrids));
      }
      for (kt_int32u i = 0; i < nGrids; i++)
      {
        CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(corrGridWidth, corrGridHeight, resolution, smearDeviation);
        Grid<kt_double>* pSearchSpaceProbs = Grid<kt_double>::CreateGrid(searchSpaceGridWidth, searchSpaceGridHeight, resolution);
        GridIndexLookup<kt_int8u>* pGridLookup = new GridIndexLookup<kt_int8u>(pCorrelationGrid);
        m_GridSets.push(std::make_shared<ScanMatcherGridSet>(pCorrelationGrid, pSearchSpaceProbs, pGridLookup));
      }
    }

    virtual ~ScanMatcherGridSetBank() {}

    std::shared_ptr<ScanMatcherGridSet> CheckOut()
    {
      std::unique_lock<std::mutex> lock(m_Mutex);
      m_Condition.wait(lock, [this]{ return !m_GridSets.empty(); });
      auto gridSet = m_GridSets.front();
      m_GridSets.pop();
      return gridSet;
    }

    void Return(std::shared_ptr<ScanMatcherGridSet> pGridSet)
    {
      {
        std::lock_guard<std::mutex> lock(m_Mutex);
        m_GridSets.push(pGridSet);
      }
      m_Condition.notify_one();
    }

  private:
    std::queue<std::shared_ptr<ScanMatcherGridSet>> m_GridSets;
    std::mutex m_Mutex;
    std::condition_variable m_Condition;
  };
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////  
  
  ScanMatcher::~ScanMatcher()
  {
    delete m_pScanMatcherGridSetBank;
  }
  
  ScanMatcher* ScanMatcher::Create(OpenMapper* pOpenMapper, kt_double searchSize, kt_double resolution, kt_double smearDeviation, kt_double rangeThreshold)
  {
    // invalid parameters
    if (resolution <= 0)
    {
      return nullptr;
    }
    if (searchSize <= 0)
    {
      return nullptr;
    }
    if (smearDeviation < 0)
    {
      return nullptr;
    }
    if (rangeThreshold <= 0)
    {
      return nullptr;
    }
    
    assert(math::DoubleEqual(math::Round(searchSize / resolution), (searchSize / resolution)));
    
    // calculate search space in grid coordinates
    kt_int32u searchSpaceSideSize = static_cast<kt_int32u>(math::Round(searchSize / resolution) + 1);
    
    // compute requisite size of correlation grid (pad grid so that scan points can't fall off the grid
    // if a scan is on the border of the search space)
    kt_int32u pointReadingMargin = static_cast<kt_int32u>(ceil(rangeThreshold / resolution));
    
    kt_int32s gridSize = searchSpaceSideSize + 2 * pointReadingMargin;
    
    // create correlation grid
    assert(gridSize % 2 == 1);
    CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(gridSize, gridSize, resolution, smearDeviation);

    // create search space probabilities
    Grid<kt_double>* pSearchSpaceProbs = Grid<kt_double>::CreateGrid(searchSpaceSideSize, searchSpaceSideSize, resolution);
    
    GridIndexLookup<kt_int8u>* pGridLookup = new GridIndexLookup<kt_int8u>(pCorrelationGrid);
    
    ScanMatcher* pScanMatcher = new ScanMatcher(pOpenMapper);
    pScanMatcher->m_pScanMatcherGridSet = std::make_shared<ScanMatcherGridSet>(pCorrelationGrid, pSearchSpaceProbs, pGridLookup);
    
    if (pOpenMapper->IsMultiThreaded())
    {
      pScanMatcher->m_pScanMatcherGridSetBank = new ScanMatcherGridSetBank(10, gridSize, gridSize, resolution, smearDeviation, searchSpaceSideSize, searchSpaceSideSize);
    }

    return pScanMatcher;
  }
  
  kt_double ScanMatcher::MatchScan(LocalizedLaserScan* pScan, const LocalizedLaserScanList& rBaseScans, Pose2& rMean, Matrix3& rCovariance, kt_bool doPenalize, kt_bool doRefineMatch)
  {
    std::shared_ptr<ScanMatcherGridSet> pScanMatcherGridSet;

    if (m_pOpenMapper->IsMultiThreaded())
    {
      pScanMatcherGridSet = m_pScanMatcherGridSetBank->CheckOut();
    }
    else
    {
      pScanMatcherGridSet = m_pScanMatcherGridSet;
    }

    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid.get();
    Grid<kt_double>* pSearchSpaceProbs = pScanMatcherGridSet->m_pSearchSpaceProbs.get();
    
    ///////////////////////////////////////
    // set scan pose to be center of grid
    
    // 1. get scan position
    Pose2 scanPose = pScan->GetSensorPose();
    
    // scan has no readings; cannot do scan matching
    // best guess of pose is based off of adjusted odometer reading
    if (pScan->GetPointReadings(true).size() == 0)
    {
      rMean = scanPose;
      
      // maximum covariance
      rCovariance(0, 0) = MAX_VARIANCE; // XX
      rCovariance(1, 1) = MAX_VARIANCE; // YY
      rCovariance(2, 2) = 4 * math::Square(m_pOpenMapper->m_pCoarseAngleResolution->GetValue()); // TH*TH
      
      if (m_pOpenMapper->IsMultiThreaded())
      {
        m_pScanMatcherGridSetBank->Return(pScanMatcherGridSet);
      }

      return 0.0;
    }
    
    // 2. get size of grid
    Rectangle2<kt_int32s> roi = pCorrelationGrid->GetROI();
    
    // 3. compute offset (in meters - lower left corner)
    Vector2d offset;
    offset.SetX(scanPose.GetX() - (0.5 * (roi.GetWidth() - 1) * pCorrelationGrid->GetResolution()));
    offset.SetY(scanPose.GetY() - (0.5 * (roi.GetHeight() - 1) * pCorrelationGrid->GetResolution()));
    
    // 4. set offset
    pCorrelationGrid->GetCoordinateConverter()->SetOffset(offset);
    
    ///////////////////////////////////////
    
    // set up correlation grid
    AddScansNew(pCorrelationGrid, rBaseScans, scanPose.GetPosition());
    
    // compute how far to search in each direction
    Vector2d searchDimensions(pSearchSpaceProbs->GetWidth(), pSearchSpaceProbs->GetHeight());
    Vector2d coarseSearchOffset(0.5 * (searchDimensions.GetX() - 1) * pCorrelationGrid->GetResolution(), 0.5 * (searchDimensions.GetY() - 1) * pCorrelationGrid->GetResolution());
    
    // a coarse search only checks half the cells in each dimension
    Vector2d coarseSearchResolution(2 * pCorrelationGrid->GetResolution(), 2 * pCorrelationGrid->GetResolution());
    
    // actual scan-matching
    kt_double bestResponse = CorrelateScan(pScanMatcherGridSet.get(), pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, m_pOpenMapper->m_pCoarseSearchAngleOffset->GetValue(), m_pOpenMapper->m_pCoarseAngleResolution->GetValue(), doPenalize, rMean, rCovariance, false);
    
    if (m_pOpenMapper->m_pUseResponseExpansion->GetValue() == true)
    {
      if (math::DoubleEqual(bestResponse, 0.0))
      {
#ifdef KARTO_DEBUG
        std::cout << "Mapper Info: Expanding response search space!" << std::endl;
#endif
        // try and increase search angle offset with 20 degrees and do another match
        kt_double newSearchAngleOffset = m_pOpenMapper->m_pCoarseSearchAngleOffset->GetValue();
        for (kt_int32u i = 0; i < 3; i++)
        {
          newSearchAngleOffset += math::DegreesToRadians(20);
          
          bestResponse = CorrelateScan(pScanMatcherGridSet.get(), pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, newSearchAngleOffset, m_pOpenMapper->m_pCoarseAngleResolution->GetValue(), doPenalize, rMean, rCovariance, false);
          
          if (math::DoubleEqual(bestResponse, 0.0) == false)
          {
            break;
          }
        }
        
#ifdef KARTO_DEBUG
        if (math::DoubleEqual(bestResponse, 0.0))
        {
          std::cout << "Mapper Warning: Unable to calculate response!" << std::endl;
        }
#endif
      }
    }
    
    if (doRefineMatch)
    {
      Vector2d fineSearchOffset(coarseSearchResolution * 0.5);
      Vector2d fineSearchResolution(pCorrelationGrid->GetResolution(), pCorrelationGrid->GetResolution());
      bestResponse = CorrelateScan(pScanMatcherGridSet.get(), pScan, rMean, fineSearchOffset, fineSearchResolution, 0.5 * m_pOpenMapper->m_pCoarseAngleResolution->GetValue(), m_pOpenMapper->m_pFineSearchAngleOffset->GetValue(), doPenalize, rMean, rCovariance, true);
    }
    
#ifdef KARTO_DEBUG
    std::cout << "  BEST POSE = " << rMean << " BEST RESPONSE = " << bestResponse << ",  VARIANCE = " << rCovariance(0, 0) << ", " << rCovariance(1, 1) << std::endl;
#endif
    
    assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

    if (m_pOpenMapper->IsMultiThreaded())
    {
      m_pScanMatcherGridSetBank->Return(pScanMatcherGridSet);
    }

    return bestResponse;
  }
  
  
  kt_double ScanMatcher::CorrelateScan(ScanMatcherGridSet* pScanMatcherGridSet, LocalizedLaserScan* pScan, const Pose2& rSearchCenter, const Vector2d& rSearchSpaceOffset, const Vector2d& rSearchSpaceResolution,
                                       kt_double searchAngleOffset, kt_double searchAngleResolution,	kt_bool doPenalize, Pose2& rMean, Matrix3& rCovariance, kt_bool doingFineMatch)
  {
    assert(searchAngleResolution != 0.0);

    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid.get();
    Grid<kt_double>* pSearchSpaceProbs = pScanMatcherGridSet->m_pSearchSpaceProbs.get();
    GridIndexLookup<kt_int8u>* pGridLookup = pScanMatcherGridSet->m_pGridLookup;

    // setup lookup arrays
    pGridLookup->ComputeOffsets(pScan, rSearchCenter.GetHeading(), searchAngleOffset, searchAngleResolution);
    
    // only initialize probability grid if computing positional covariance (during coarse match)
    if (!doingFineMatch)
    {
      pSearchSpaceProbs->Clear();
      
      // position search grid - finds lower left corner of search grid
      Vector2d offset(rSearchCenter.GetPosition() - rSearchSpaceOffset);
      pSearchSpaceProbs->GetCoordinateConverter()->SetOffset(offset);
    }
    
    // calculate position arrays
    
    kt_int32u nX = static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetX() * 2.0 / rSearchSpaceResolution.GetX()) + 1);
    std::vector<kt_double> xPoses(nX), newPositionsX(nX), squaresX(nX);
    kt_double startX = -rSearchSpaceOffset.GetX();
    for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
    {
      kt_double x = startX + xIndex * rSearchSpaceResolution.GetX();
      xPoses[xIndex] = x;
      newPositionsX[xIndex] = rSearchCenter.GetX() + x;
      squaresX[xIndex] = math::Square(x);
    }
    assert(math::DoubleEqual(xPoses.back(), -startX));
    
    kt_int32u nY = static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetY() * 2.0 / rSearchSpaceResolution.GetY()) + 1);
    std::vector<kt_double> yPoses(nY), newPositionsY(nY), squaresY(nY);
    kt_double startY = -rSearchSpaceOffset.GetY();
    for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
    {
      kt_double y = startY + yIndex * rSearchSpaceResolution.GetY();
      yPoses[yIndex] = y;
      newPositionsY[yIndex] = rSearchCenter.GetY() + y;
      squaresY[yIndex] = math::Square(y);
    }
    assert(math::DoubleEqual(yPoses.back(), -startY));
    
    // calculate pose response array size
    kt_int32u nAngles = static_cast<kt_int32u>(math::Round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);
    std::vector<kt_double> angles(nAngles);
    kt_double angle = 0.0;
    kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
    for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * searchAngleResolution;
      angles[angleIndex] = angle;
    }
    assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
    
    // allocate array
    kt_int32u poseResponseSize = nX * nY * nAngles;
    std::vector<std::pair<kt_double, Pose2> > poseResponses = std::vector<std::pair<kt_double, Pose2> >(poseResponseSize);
    
    Vector2i startGridPoint = pCorrelationGrid->WorldToGrid(Vector2d(rSearchCenter.GetX() + startX, rSearchCenter.GetY() + startY));
    
    if (m_pOpenMapper->IsMultiThreaded())
    {
      unsigned int numThreads = std::thread::hardware_concurrency();
      if (numThreads == 0) numThreads = 4;

      std::vector<std::thread> threads;
      threads.reserve(numThreads);

      auto worker = [&](kt_int32u yStart, kt_int32u yEnd) {
        for (kt_int32u yIndex = yStart; yIndex < yEnd; yIndex++)
        {
          kt_double newPositionY = newPositionsY[yIndex];
          kt_double squareY = squaresY[yIndex];
          for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
          {
            kt_double newPositionX = newPositionsX[xIndex];
            kt_double squareX = squaresX[xIndex];
            Vector2i gridPoint = pCorrelationGrid->WorldToGrid(Vector2d(newPositionX, newPositionY));
            kt_int32s gridIndex = pCorrelationGrid->GridIndex(gridPoint);
            assert(gridIndex >= 0);
            kt_double squaredDistance = squareX + squareY;
            for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
            {
              kt_int32u poseResponseIndex = (nX * nAngles) * yIndex + nAngles * xIndex + angleIndex;
              kt_double angle = angles[angleIndex];
              kt_double response = GetResponse(pScanMatcherGridSet, angleIndex, gridIndex);
              if (doPenalize && (math::DoubleEqual(response, 0.0) == false))
              {
                kt_double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance / m_pOpenMapper->m_pDistanceVariancePenalty->GetValue());
                distancePenalty = math::Maximum(distancePenalty, m_pOpenMapper->m_pMinimumDistancePenalty->GetValue());
                kt_double squaredAngleDistance = math::Square(angle - rSearchCenter.GetHeading());
                kt_double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance / m_pOpenMapper->m_pAngleVariancePenalty->GetValue());
                anglePenalty = math::Maximum(anglePenalty, m_pOpenMapper->m_pMinimumAnglePenalty->GetValue());
                response *= (distancePenalty * anglePenalty);
              }
              poseResponses[poseResponseIndex] = std::pair<kt_double, Pose2>(response, Pose2(newPositionX, newPositionY, math::NormalizeAngle(angle)));
            }
          }
        }
      };

      kt_int32u rowsPerThread = nY / numThreads;
      kt_int32u remainder = nY % numThreads;
      kt_int32u start = 0;
      for (unsigned int t = 0; t < numThreads; t++)
      {
        kt_int32u end = start + rowsPerThread + (t < remainder ? 1 : 0);
        if (start < end)
          threads.emplace_back(worker, start, end);
        start = end;
      }
      for (auto& thread : threads)
        thread.join();
    }
    else
    {
      kt_int32u poseResponseCounter = 0;
      for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
      {
        kt_double newPositionY = newPositionsY[yIndex];
        kt_double squareY = squaresY[yIndex];

        for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
        {
          kt_double newPositionX = newPositionsX[xIndex];
          kt_double squareX = squaresX[xIndex];

          Vector2i gridPoint = pCorrelationGrid->WorldToGrid(Vector2d(newPositionX, newPositionY));
          kt_int32s gridIndex = pCorrelationGrid->GridIndex(gridPoint);
          assert(gridIndex >= 0);

          kt_double squaredDistance = squareX + squareY;
          for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
          {
            kt_double angle = angles[angleIndex];

            kt_double response = GetResponse(pScanMatcherGridSet, angleIndex, gridIndex);
            if (doPenalize && (math::DoubleEqual(response, 0.0) == false))
            {
              // simple model (approximate Gaussian) to take odometry into account

              kt_double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance / m_pOpenMapper->m_pDistanceVariancePenalty->GetValue());
              distancePenalty = math::Maximum(distancePenalty, m_pOpenMapper->m_pMinimumDistancePenalty->GetValue());

              kt_double squaredAngleDistance = math::Square(angle - rSearchCenter.GetHeading());
              kt_double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance / m_pOpenMapper->m_pAngleVariancePenalty->GetValue());
              anglePenalty = math::Maximum(anglePenalty, m_pOpenMapper->m_pMinimumAnglePenalty->GetValue());

              response *= (distancePenalty * anglePenalty);
            }

            // store response and pose
            poseResponses[poseResponseCounter] = std::pair<kt_double, Pose2>(response, Pose2(newPositionX, newPositionY, math::NormalizeAngle(angle)));
            poseResponseCounter++;
          }
        }
      }

      assert(poseResponseSize == poseResponseCounter);
    }
    
    // find value of best response (in [0; 1])
    kt_double bestResponse = -1;
    for (kt_int32u i = 0; i < poseResponseSize; i++)
    {
      bestResponse = math::Maximum(bestResponse, poseResponses[i].first);
      
      // will compute positional covariance, save best relative probability for each cell
      if (!doingFineMatch)
      {
        const Pose2& rPose = poseResponses[i].second;
        Vector2i grid = pSearchSpaceProbs->WorldToGrid(rPose.GetPosition());
        
        kt_double* ptr = (kt_double*)pSearchSpaceProbs->GetDataPointer(grid);
        if (ptr == nullptr)
        {
          throw Exception("Mapper FATAL ERROR - Index out of range in probability search!");
        }
        
        *ptr = math::Maximum(poseResponses[i].first, *ptr);
      }
    }
    
    // average all poses with same highest response
    Vector2d averagePosition;
    kt_double thetaX = 0.0;
    kt_double thetaY = 0.0;
    kt_int32s averagePoseCount = 0;
    for (kt_int32u i = 0; i < poseResponseSize; i++)
    {
      if (math::DoubleEqual(poseResponses[i].first, bestResponse))
      {
        averagePosition += poseResponses[i].second.GetPosition();
        
        kt_double heading = poseResponses[i].second.GetHeading();
        thetaX += cos(heading);
        thetaY += sin(heading);
        
        averagePoseCount++;
      }
    }
    
    Pose2 averagePose;
    if (averagePoseCount > 0)
    {
      averagePosition /= averagePoseCount;
      
      thetaX /= averagePoseCount;
      thetaY /= averagePoseCount;
      
      averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
    }
    else
    {
      throw Exception("Mapper FATAL ERROR - Unable to find best position");
    }
    
#ifdef KARTO_DEBUG
    std::cout << "bestPose: " << averagePose << std::endl;
    std::cout << "bestResponse: " << bestResponse << std::endl;
#endif
    
    if (!doingFineMatch)
    {
      ComputePositionalCovariance(pSearchSpaceProbs, averagePose, bestResponse, rSearchCenter, rSearchSpaceOffset, rSearchSpaceResolution, searchAngleResolution, rCovariance);
    }
    else
    {
      ComputeAngularCovariance(pScanMatcherGridSet, averagePose, bestResponse, rSearchCenter, searchAngleOffset, searchAngleResolution, rCovariance);
    }
    
    rMean = averagePose;
    
#ifdef KARTO_DEBUG
    std::cout << "bestPose: " << averagePose << std::endl;
#endif
    
    if (bestResponse > 1.0)
    {
      bestResponse = 1.0;
    }
    
    assert(math::InRange(bestResponse, 0.0, 1.0));
    assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));
    
    return bestResponse;
  }
  
  void ScanMatcher::ComputePositionalCovariance(Grid<kt_double>* pSearchSpaceProbs, const Pose2& rBestPose, kt_double bestResponse,
                                                const Pose2& rSearchCenter, const Vector2d& rSearchSpaceOffset,
                                                const Vector2d& rSearchSpaceResolution, kt_double searchAngleResolution, Matrix3& rCovariance)
  {
    // reset covariance to identity matrix
    rCovariance.SetToIdentity();
    
    // if best response is vary small return max variance
    if (bestResponse < KT_TOLERANCE)
    {
      rCovariance(0, 0) = MAX_VARIANCE; // XX
      rCovariance(1, 1) = MAX_VARIANCE; // YY
      rCovariance(2, 2) = 4 * math::Square(searchAngleResolution); // TH*TH
      
      return;
    }
    
    kt_double accumulatedVarianceXX = 0;
    kt_double accumulatedVarianceXY = 0;
    kt_double accumulatedVarianceYY = 0;
    kt_double norm = 0;
    
    kt_double dx = rBestPose.GetX() - rSearchCenter.GetX();
    kt_double dy = rBestPose.GetY() - rSearchCenter.GetY();
    
    kt_double offsetX = rSearchSpaceOffset.GetX();
    kt_double offsetY = rSearchSpaceOffset.GetY();
    
    kt_int32u nX = static_cast<kt_int32u>(math::Round(offsetX * 2.0 / rSearchSpaceResolution.GetX()) + 1);
    kt_double startX = -offsetX;
    assert(math::DoubleEqual(startX + (nX - 1) * rSearchSpaceResolution.GetX(), -startX));
    
    kt_int32u nY = static_cast<kt_int32u>(math::Round(offsetY * 2.0 / rSearchSpaceResolution.GetY()) + 1);
    kt_double startY = -offsetY;
    assert(math::DoubleEqual(startY + (nY - 1) * rSearchSpaceResolution.GetY(), -startY));
    
    for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
    {
      kt_double y = startY + yIndex * rSearchSpaceResolution.GetY();
      
      for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
      {
        kt_double x = startX + xIndex * rSearchSpaceResolution.GetX();
        
        Vector2i gridPoint = pSearchSpaceProbs->WorldToGrid(Vector2d(rSearchCenter.GetX() + x, rSearchCenter.GetY() + y));
        kt_double response = *(pSearchSpaceProbs->GetDataPointer(gridPoint));
        
        // response is not a low response
        if (response >= (bestResponse - 0.1))
        {
          norm += response;
          accumulatedVarianceXX += (math::Square(x - dx) * response);
          accumulatedVarianceXY += ((x - dx) * (y - dy) * response);
          accumulatedVarianceYY += (math::Square(y - dy) * response);
        }
      }
    }
    
    if (norm > KT_TOLERANCE)
    {
      kt_double varianceXX = accumulatedVarianceXX / norm;
      kt_double varianceXY = accumulatedVarianceXY / norm;
      kt_double varianceYY = accumulatedVarianceYY / norm;
      kt_double varianceTHTH = 4 * math::Square(searchAngleResolution);
      
      // lower-bound variances so that they are not too small;
      // ensures that links are not too tight
      kt_double minVarianceXX = 0.1 * math::Square(rSearchSpaceResolution.GetX());
      kt_double minVarianceYY = 0.1 * math::Square(rSearchSpaceResolution.GetY());
      varianceXX = math::Maximum(varianceXX, minVarianceXX);
      varianceYY = math::Maximum(varianceYY, minVarianceYY);
      
      // increase variance for poorer responses
      kt_double multiplier = 1.0 / bestResponse;
      rCovariance(0, 0) = varianceXX * multiplier;
      rCovariance(0, 1) = varianceXY * multiplier;
      rCovariance(1, 0) = varianceXY * multiplier;
      rCovariance(1, 1) = varianceYY * multiplier;
      rCovariance(2, 2) = varianceTHTH; // this value will be set in ComputeAngularCovariance
    }
    
    // if values are 0, set to MAX_VARIANCE
    // values might be 0 if points are too sparse and thus don't hit other points
    if (math::DoubleEqual(rCovariance(0, 0), 0.0))
    {
      rCovariance(0, 0) = MAX_VARIANCE;
    }
    
    if (math::DoubleEqual(rCovariance(1, 1), 0.0))
    {
      rCovariance(1, 1) = MAX_VARIANCE;
    }
  }
  
  void ScanMatcher::ComputeAngularCovariance(ScanMatcherGridSet* pScanMatcherGridSet, const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                             kt_double searchAngleOffset, kt_double searchAngleResolution, Matrix3& rCovariance)
  {
    // NOTE: do not reset covariance matrix
    
    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid.get();
    
    // normalize angle difference
    kt_double bestAngle = math::NormalizeAngleDifference(rBestPose.GetHeading(), rSearchCenter.GetHeading());
    
    Vector2i gridPoint = pCorrelationGrid->WorldToGrid(rBestPose.GetPosition());
    kt_int32s gridIndex = pCorrelationGrid->GridIndex(gridPoint);
    
    kt_int32u nAngles = static_cast<kt_int32u>(math::Round(searchAngleOffset * 2 / searchAngleResolution) + 1);
    
    kt_double angle = 0.0;
    kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
    
    kt_double norm = 0.0;
    kt_double accumulatedVarianceThTh = 0.0;
    for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * searchAngleResolution;
      kt_double response = GetResponse(pScanMatcherGridSet, angleIndex, gridIndex);
      
      // response is not a low response
      if (response >= (bestResponse - 0.1))
      {
        norm += response;
        accumulatedVarianceThTh += (math::Square(angle - bestAngle) * response);
      }
    }
    assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
    
    if (norm > KT_TOLERANCE)
    {
      if (accumulatedVarianceThTh < KT_TOLERANCE)
      {
        accumulatedVarianceThTh = math::Square(searchAngleResolution);
      }
      
      accumulatedVarianceThTh /= norm;
    }
    else
    {
      accumulatedVarianceThTh = 1000 * math::Square(searchAngleResolution);
    }
    
    rCovariance(2, 2) = accumulatedVarianceThTh;
  }
  
  void ScanMatcher::AddScans(CorrelationGrid* pCorrelationGrid, const LocalizedLaserScanList& rScans, const Vector2d& rViewPoint)
  {
    pCorrelationGrid->Clear();
    
    // add all scans to grid
    for (const auto& pScan : rScans)
    {
      AddScan(pCorrelationGrid, pScan, rViewPoint);
    }
  }
  
  void ScanMatcher::AddScansNew(CorrelationGrid* pCorrelationGrid, const LocalizedLaserScanList& rScans, const Vector2d& rViewPoint)
  {
    pCorrelationGrid->Clear();

    kt_int32s index = 0;
    kt_size_t nScans = rScans.size();
    Vector2dList* pValidPoints = new Vector2dList[nScans];

    // first find all valid points
    for (const auto& pScan : rScans)
    {
      pValidPoints[index++] = FindValidPoints(pScan, rViewPoint);
    }

    // then add all valid points to correlation grid
    for (kt_size_t i = 0; i < nScans; i++)
    {
      AddScanNew(pCorrelationGrid, pValidPoints[i]);
    }

    delete[] pValidPoints;
  }

  void ScanMatcher::AddScan(CorrelationGrid* pCorrelationGrid, LocalizedLaserScan* pScan, const Vector2d& rViewPoint, kt_bool doSmear)
  {
    Vector2dList validPoints = FindValidPoints(pScan, rViewPoint);
    
    // put in all valid points
    for (const auto& point : validPoints)
    {
      Vector2i gridPoint = pCorrelationGrid->WorldToGrid(point);
      if (!math::IsUpTo(gridPoint.GetX(), pCorrelationGrid->GetROI().GetWidth()) || !math::IsUpTo(gridPoint.GetY(), pCorrelationGrid->GetROI().GetHeight()))
      {
        // point not in grid
        continue;
      }

      int gridIndex = pCorrelationGrid->GridIndex(gridPoint);

      // set grid cell as occupied
      if (pCorrelationGrid->GetDataPointer()[gridIndex] == GridStates_Occupied)
      {
        // value already set
        continue;
      }

      pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;

      // smear grid
      if (doSmear == true)
      {
        pCorrelationGrid->SmearPoint(gridPoint);
      }
    }
  }

  void ScanMatcher::AddScanNew(CorrelationGrid* pCorrelationGrid, const Vector2dList& rValidPoints, kt_bool doSmear)
  {
    // put in all valid points
    for (const auto& point : rValidPoints)
    {
      Vector2i gridPoint = pCorrelationGrid->WorldToGrid(point);
      if (!math::IsUpTo(gridPoint.GetX(), pCorrelationGrid->GetROI().GetWidth()) || !math::IsUpTo(gridPoint.GetY(), pCorrelationGrid->GetROI().GetHeight()))
      {
        // point not in grid
        continue;
      }

      int gridIndex = pCorrelationGrid->GridIndex(gridPoint);

      // set grid cell as occupied
      if (pCorrelationGrid->GetDataPointer()[gridIndex] == GridStates_Occupied)
      {
        // value already set
        continue;
      }

      pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;

      // smear grid
      if (doSmear == true)
      {
        pCorrelationGrid->SmearPoint(gridPoint);
      }
    }
  }

  Vector2dList ScanMatcher::FindValidPoints(LocalizedLaserScan* pScan, const Vector2d& rViewPoint)
  {
    const Vector2dList& rPointReadings = pScan->GetPointReadings(true);
    
    // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
    const kt_double minSquareDistance = math::Square(0.1); // in m^2

    // this iterator lags from the main iterator adding points only when the points are on
    // the same side as the viewpoint
    kt_size_t trailingPointIndex = 0;
    Vector2dList validPoints;

    Vector2d firstPoint;
    kt_bool firstTime = true;
    for (kt_size_t i = 0; i < rPointReadings.size(); i++)
    {
      Vector2d currentPoint = rPointReadings[i];

      if (firstTime)
      {
        firstPoint = currentPoint;
        firstTime = false;
      }

      Vector2d delta = firstPoint - currentPoint;
      if (delta.SquaredLength() > minSquareDistance)
      {
        // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
        // Which computes the direction of rotation, if the rotation is counterclock
        // wise then we are looking at data we should keep. If it's negative rotation
        // we should not included in in the matching
        // have enough distance, check viewpoint
        double a = rViewPoint.GetY() - firstPoint.GetY();
        double b = firstPoint.GetX() - rViewPoint.GetX();
        double c = firstPoint.GetY() * rViewPoint.GetX() - firstPoint.GetX() * rViewPoint.GetY();
        double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;

        // reset beginning point
        firstPoint = currentPoint;

        if (ss < 0.0)	// wrong side, skip and keep going
        {
          trailingPointIndex = i;
        }
        else
        {
          for (; trailingPointIndex < i; trailingPointIndex++)
          {
            validPoints.push_back(rPointReadings[trailingPointIndex]);
          }
        }
      }
    }

    return validPoints;
  }  
  
  kt_double ScanMatcher::GetResponse(ScanMatcherGridSet* pScanMatcherGridSet, kt_int32u angleIndex, kt_int32s gridPositionIndex)
  {
    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid.get();
    GridIndexLookup<kt_int8u>* pGridLookup = pScanMatcherGridSet->m_pGridLookup;
    
    kt_double response = 0.0;
    
    // add up value for each point
    kt_int8u* pByte = pCorrelationGrid->GetDataPointer() + gridPositionIndex;
    
    const LookupArray* pOffsets = pGridLookup->GetLookupArray(angleIndex);
    assert(pOffsets != nullptr);
    
    // get number of points in offset list
    kt_int32u nPoints = pOffsets->GetSize();
    if (nPoints == 0)
    {
      return response;
    }
    
    // calculate response
    kt_int32s* pAngleIndexPointer = pOffsets->GetArrayPointer();
    for (kt_int32u i = 0; i < nPoints; i++)
    {
      // ignore points that fall off the grid
      kt_int32s pointGridIndex = gridPositionIndex + pAngleIndexPointer[i];
      if (!math::IsUpTo(pointGridIndex, pCorrelationGrid->GetDataSize()))
      {
        continue;
      }

      // uses index offsets to efficiently find location of point in the grid
      response += pByte[pAngleIndexPointer[i]];
    }
    
    // normalize response
    response /= (nPoints * GridStates_Occupied);
    assert(fabs(response) <= 1.0);
    
    return response;
  }
  
  CorrelationGrid* ScanMatcher::GetCorrelationGrid() const
  {
    if (m_pOpenMapper->IsMultiThreaded())
    {
      throw Exception("Correlation grid only available in single-threaded mode");
    }
    else
    {
      return m_pScanMatcherGridSet->m_pCorrelationGrid.get();
    }
  }
  
  Grid<kt_double>* ScanMatcher::GetSearchGrid() const
  {
    if (m_pOpenMapper->IsMultiThreaded())
    {
      throw Exception("Search grid only available in single-threaded mode");
    }
    else
    {
      return m_pScanMatcherGridSet->m_pSearchSpaceProbs.get();
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class BreadthFirstTraversal : public GraphTraversal<T>
  {
  public:
    /**
     * Constructs a breadth-first traverser for the given graph
     */
    BreadthFirstTraversal(Graph<T>* pGraph)
      : GraphTraversal<T>(pGraph)
    {
    }

    /**
     * Destructor
     */
    virtual ~BreadthFirstTraversal()
    {
    }

  public:
    /**
     * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
     * @param pStartVertex
     * @param pVisitor
     * @return visited vertices
     */
    virtual std::vector<T> Traverse(Vertex<T>* pStartVertex, Visitor<T>* pVisitor)
    {
      std::queue<Vertex<T>*> toVisit;
      std::set<Vertex<T>*> seenVertices;
      std::vector<Vertex<T>*> validVertices;

      toVisit.push(pStartVertex);
      seenVertices.insert(pStartVertex);

      do
      {
        Vertex<T>* pNext = toVisit.front();
        toVisit.pop();

        if (pVisitor->Visit(pNext))
        {
          // vertex is valid, explore neighbors
          validVertices.push_back(pNext);

          std::vector<Vertex<T>*> adjacentVertices = pNext->GetAdjacentVertices();
          for (auto& pAdjacent : adjacentVertices)
          {
            // adjacent vertex has not yet been seen, add to queue for processing
            if (seenVertices.find(pAdjacent) == seenVertices.end())
            {
              toVisit.push(pAdjacent);
              seenVertices.insert(pAdjacent);
            }
          }
        }
      } while (toVisit.empty() == false);

      std::vector<T> objects;
      for (auto& pVertex : validVertices)
      {
        objects.push_back(pVertex->GetVertexObject());
      }

      return objects;
    }

  }; // class BreadthFirstTraversal

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class NearScanVisitor : public Visitor<LocalizedObject*>
  {
  public:
    NearScanVisitor(LocalizedLaserScan* pScan, kt_double maxDistance, kt_bool useScanBarycenter)
      : m_MaxDistanceSquared(math::Square(maxDistance))
      , m_UseScanBarycenter(useScanBarycenter)
    {
      m_CenterPose = pScan->GetReferencePose(m_UseScanBarycenter);
    }

    virtual kt_bool Visit(Vertex<LocalizedObject*>* pVertex)
    {
      LocalizedObject* pObject = pVertex->GetVertexObject();

      LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
      
      // object is not a scan or wasn't scan matched, ignore
      if (pScan == nullptr)
      {
        return false;
      }
      
      Pose2 pose = pScan->GetReferencePose(m_UseScanBarycenter);

      kt_double squaredDistance = pose.GetPosition().SquaredDistance(m_CenterPose.GetPosition());
      return (squaredDistance <= m_MaxDistanceSquared - KT_TOLERANCE);
    }

  protected:
    Pose2 m_CenterPose;
    kt_double m_MaxDistanceSquared;
    kt_bool m_UseScanBarycenter;

  }; // NearScanVisitor

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  MapperGraph::MapperGraph(OpenMapper* pOpenMapper, kt_double rangeThreshold)
    : m_pOpenMapper(pOpenMapper)
  {
    m_pLoopScanMatcher = ScanMatcher::Create(pOpenMapper, m_pOpenMapper->m_pLoopSearchSpaceDimension->GetValue(), m_pOpenMapper->m_pLoopSearchSpaceResolution->GetValue(), m_pOpenMapper->m_pLoopSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
    assert(m_pLoopScanMatcher);
    
    m_pTraversal = new BreadthFirstTraversal<LocalizedObject*>(this);
  }
  
  MapperGraph::~MapperGraph()
  {
    delete m_pLoopScanMatcher;
    m_pLoopScanMatcher = nullptr;
    
    delete m_pTraversal;
    m_pTraversal = nullptr;
  }
  
  void MapperGraph::AddVertex(LocalizedObject* pObject)
  {
    assert(pObject);

    if (pObject == nullptr)
    {
      return;
    }
    
    Vertex<LocalizedObject*>* pVertex = new Vertex<LocalizedObject*>(pObject);
    Graph<LocalizedObject*>::AddVertex(pVertex);
    if (m_pOpenMapper->m_pScanSolver != nullptr)
    {
      m_pOpenMapper->m_pScanSolver->AddNode(pVertex);
    }
  }

  void MapperGraph::AddEdges(LocalizedObject* pObject)
  {
    // loose "spring"
    Matrix3 covariance;
    covariance(0, 0) = MAX_VARIANCE;
    covariance(1, 1) = MAX_VARIANCE;
    covariance(2, 2) = MAX_VARIANCE;
    
    LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
    if (pScan != nullptr)
    {      
      AddEdges(pScan, covariance);
    }
    else
    {
      MapperSensorManager* pSensorManager = m_pOpenMapper->m_pMapperSensorManager;      
      const Identifier& rSensorName = pObject->GetSensorIdentifier();
      
      LocalizedLaserScan* pLastScan = pSensorManager->GetLastScan(rSensorName);
      if (pLastScan != nullptr)
      {
        LinkObjects(pLastScan, pObject, pObject->GetCorrectedPose(), covariance);
      }
    }
  }

  void MapperGraph::AddEdges(LocalizedLaserScan* pScan, const Matrix3& rCovariance)
  {
    MapperSensorManager* pSensorManager = m_pOpenMapper->m_pMapperSensorManager;
    
    const Identifier& rSensorName = pScan->GetSensorIdentifier();
    
    Pose2List means;
    std::vector<Matrix3> covariances;

    LocalizedLaserScan* pLastScan = pSensorManager->GetLastScan(rSensorName);
    if (pLastScan == nullptr)
    {
      // first scan (link to first scan of other robots)

      assert(pSensorManager->GetScans(rSensorName).size() == 1);

      std::vector<Identifier> sensorNames = pSensorManager->GetSensorNames();
      for (const auto& rCandidateSensorName : sensorNames)
      {
        // skip if candidate sensor is the same or other sensor has no scans
        if ((rCandidateSensorName == rSensorName) || (pSensorManager->GetScans(rCandidateSensorName).empty()))
        {
          continue;
        }

        Pose2 bestPose;
        Matrix3 covariance;
        kt_double response = m_pOpenMapper->m_pSequentialScanMatcher->MatchScan(pScan, pSensorManager->GetScans(rCandidateSensorName), bestPose, covariance);
        LinkObjects(pSensorManager->GetScans(rCandidateSensorName)[0], pScan, bestPose, covariance);

        // only add to means and covariances if response was high "enough"
        if (response > m_pOpenMapper->m_pLinkMatchMinimumResponseFine->GetValue())
        {
          means.push_back(bestPose);
          covariances.push_back(covariance);
        }
      }
    }
    else
    {
      // link to previous scan
      LinkObjects(pLastScan, pScan, pScan->GetSensorPose(), rCovariance);

      // link to running scans
      Pose2 scanPose = pScan->GetSensorPose();
      means.push_back(scanPose);
      covariances.push_back(rCovariance);
      LinkChainToScan(pSensorManager->GetRunningScans(rSensorName), pScan, scanPose, rCovariance);
    }

    // link to other near chains (chains that include new scan are invalid)
    LinkNearChains(pScan, means, covariances);

    if (!means.empty())
    {
      pScan->SetSensorPose(ComputeWeightedMean(means, covariances));
    }
  }
  
  kt_bool MapperGraph::TryCloseLoop(LocalizedLaserScan* pScan, const Identifier& rSensorName)
  {
    kt_bool loopClosed = false;
    
    kt_int32u scanIndex = 0;
    
    LocalizedLaserScanList candidateChain = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);
    
    while (!candidateChain.empty())
    {
#ifdef KARTO_DEBUG2
      std::cout << "Candidate chain for " << pScan->GetStateId() << ": [ ";
      for (const auto& chainScan : candidateChain)
      {
        std::cout << chainScan->GetStateId() << " ";
      }
      std::cout << "]" << std::endl;
#endif
        
      Pose2 bestPose;
      Matrix3 covariance;
      kt_double coarseResponse = m_pLoopScanMatcher->MatchScan(pScan, candidateChain, bestPose, covariance, false, false);
      
      StringBuilder message;
      message << "COARSE RESPONSE: " << coarseResponse << " (> " << m_pOpenMapper->m_pLoopMatchMinimumResponseCoarse->GetValue() << ")\n";
      message << "            var: " << covariance(0, 0) << ",  " << covariance(1, 1) << " (< " << m_pOpenMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue() << ")";
       
      MapperEventArguments eventArguments(message.ToString());
      m_pOpenMapper->Message.Notify(this, eventArguments);
      
      if (((coarseResponse > m_pOpenMapper->m_pLoopMatchMinimumResponseCoarse->GetValue()) &&
           (covariance(0, 0) < m_pOpenMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue()) &&
           (covariance(1, 1) < m_pOpenMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue()))
          ||
          // be more lenient if the variance is really small
          ((coarseResponse > 0.9 * m_pOpenMapper->m_pLoopMatchMinimumResponseCoarse->GetValue()) &&
           (covariance(0, 0) < 0.01 * m_pOpenMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue()) &&
           (covariance(1, 1) < 0.01 * m_pOpenMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue())))
      {
        // save for reversion
        Pose2 oldPose = pScan->GetSensorPose();
        
        pScan->SetSensorPose(bestPose);
        kt_double fineResponse = m_pOpenMapper->m_pSequentialScanMatcher->MatchScan(pScan, candidateChain, bestPose, covariance, false);
        
        message.Clear();
        message << "FINE RESPONSE: " << fineResponse << " (>" << m_pOpenMapper->m_pLoopMatchMinimumResponseFine->GetValue() << ")";
        MapperEventArguments eventArguments(message.ToString());
        m_pOpenMapper->Message.Notify(this, eventArguments);
        
        if (fineResponse < m_pOpenMapper->m_pLoopMatchMinimumResponseFine->GetValue())
        {
          // failed verification test, revert
          pScan->SetSensorPose(oldPose);
          
          MapperEventArguments eventArguments("REJECTED!");
          m_pOpenMapper->Message.Notify(this, eventArguments);
        }
        else
        {
          MapperEventArguments eventArguments1("Closing loop...");
          m_pOpenMapper->PreLoopClosed.Notify(this, eventArguments1);
          
          pScan->SetSensorPose(bestPose);
          LinkChainToScan(candidateChain, pScan, bestPose, covariance);
          CorrectPoses();

          MapperEventArguments eventArguments2("Loop closed!");
          m_pOpenMapper->PostLoopClosed.Notify(this, eventArguments2);
          
          m_pOpenMapper->ScansUpdated.Notify(this, karto::EventArguments::Empty());      

          loopClosed = true;
        }
      }
      
      candidateChain = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);
    }
    
    return loopClosed;
  }
  
  LocalizedLaserScan* MapperGraph::GetClosestScanToPose(const LocalizedLaserScanList& rScans, const Pose2& rPose) const
  {
    LocalizedLaserScan* pClosestScan = nullptr;
    kt_double bestSquaredDistance = DBL_MAX;
    
    for (const auto& scan : rScans)
    {
      Pose2 scanPose = scan->GetReferencePose(m_pOpenMapper->m_pUseScanBarycenter->GetValue());

      kt_double squaredDistance = rPose.GetPosition().SquaredDistance(scanPose.GetPosition());
      if (squaredDistance < bestSquaredDistance)
      {
        bestSquaredDistance = squaredDistance;
        pClosestScan = scan;
      }
    }
    
    return pClosestScan;
  }
    
  Edge<LocalizedObject*>* MapperGraph::AddEdge(LocalizedObject* pSourceObject, LocalizedObject* pTargetObject, kt_bool& rIsNewEdge)
  {
    // check that vertex exists
    assert(pSourceObject->GetUniqueId() < (kt_int32s)m_Vertices.size());
    assert(pTargetObject->GetUniqueId() < (kt_int32s)m_Vertices.size());
    
    Vertex<LocalizedObject*>* v1 = m_Vertices[pSourceObject->GetUniqueId()];
    Vertex<LocalizedObject*>* v2 = m_Vertices[pTargetObject->GetUniqueId()];
    
    // see if edge already exists
    for (const auto& pEdge : v1->GetEdges())
    {
      if (pEdge->GetTarget() == v2)
      {
        rIsNewEdge = false;
        return pEdge;
      }
    }
    
    Edge<LocalizedObject*>* pEdge = new Edge<LocalizedObject*>(v1, v2);
    Graph<LocalizedObject*>::AddEdge(pEdge);
    rIsNewEdge = true;
    return pEdge;
  }
  
  void MapperGraph::LinkObjects(LocalizedObject* pFromObject, LocalizedObject* pToObject, const Pose2& rMean, const Matrix3& rCovariance)
  {
    kt_bool isNewEdge = true;
    Edge<LocalizedObject*>* pEdge = AddEdge(pFromObject, pToObject, isNewEdge);
    
    // only attach link information if the edge is new
    if (isNewEdge == true)
    {
      LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pFromObject);
      if (pScan != nullptr)
      {
        pEdge->SetLabel(new LinkInfo(pScan->GetSensorPose(), rMean, rCovariance));
      }
      else
      {
        pEdge->SetLabel(new LinkInfo(pFromObject->GetCorrectedPose(), rMean, rCovariance));
      }
      if (m_pOpenMapper->m_pScanSolver != nullptr)
      {
        m_pOpenMapper->m_pScanSolver->AddConstraint(pEdge);
      }
    }
  }
  
  void MapperGraph::LinkNearChains(LocalizedLaserScan* pScan, Pose2List& rMeans, std::vector<Matrix3>& rCovariances)
  {
    const std::vector<LocalizedLaserScanList> nearChains = FindNearChains(pScan);

    if (m_pOpenMapper->IsMultiThreaded())
    {
      kt_int32s chainCount = static_cast<kt_int32s>(nearChains.size());
      std::vector<kt_bool> wasChainLinked(chainCount, false);
      Pose2List means(chainCount);
      std::vector<Matrix3> covariances(chainCount);

      unsigned int numThreads = std::thread::hardware_concurrency();
      if (numThreads == 0) numThreads = 4;

      std::vector<std::thread> threads;
      auto worker = [&](kt_int32s iStart, kt_int32s iEnd) {
        for (kt_int32s i = iStart; i < iEnd; i++)
        {
          const LocalizedLaserScanList& rChain = nearChains[i];
          if (rChain.size() < m_pOpenMapper->m_pLoopMatchMinimumChainSize->GetValue())
            continue;
          Pose2 mean;
          Matrix3 covariance;
          kt_double response = m_pOpenMapper->GetSequentialScanMatcher()->MatchScan(pScan, rChain, mean, covariance, false);
          if (response > m_pOpenMapper->m_pLinkMatchMinimumResponseFine->GetValue() - KT_TOLERANCE)
          {
            wasChainLinked[i] = true;
            means[i] = mean;
            covariances[i] = covariance;
          }
        }
      };

      kt_int32s rowsPerThread = chainCount / numThreads;
      kt_int32s remainder = chainCount % numThreads;
      kt_int32s start = 0;
      for (unsigned int t = 0; t < numThreads; t++)
      {
        kt_int32s end = start + rowsPerThread + (t < remainder ? 1 : 0);
        if (start < end)
          threads.emplace_back(worker, start, end);
        start = end;
      }
      for (auto& thread : threads)
        thread.join();

      for (kt_int32s i = 0; i < chainCount; i++)
      {
        if (wasChainLinked[i])
        {
          rMeans.push_back(means[i]);
          rCovariances.push_back(covariances[i]);
          LinkChainToScan(nearChains[i], pScan, means[i], covariances[i]);
        }
      }
    }
    else
    {
      for (const auto& chain : nearChains)
      {
#ifdef KARTO_DEBUG2
        std::cout << "Near chain for " << pScan->GetStateId() << ": [ ";
        for (const auto& chainScan : chain)
        {
          std::cout << chainScan->GetStateId() << " ";
        }
        std::cout << "]: ";
#endif

        if (chain.size() < m_pOpenMapper->m_pLoopMatchMinimumChainSize->GetValue())
        {
#ifdef KARTO_DEBUG2
          std::cout << chain.size() << "(< " << m_pOpenMapper->m_pLoopMatchMinimumChainSize->GetValue() << ") REJECTED" << std::endl;
#endif
          continue;
        }

        Pose2 mean;
        Matrix3 covariance;
        // match scan against "near" chain
        kt_double response = m_pOpenMapper->m_pSequentialScanMatcher->MatchScan(pScan, chain, mean, covariance, false);
        if (response > m_pOpenMapper->m_pLinkMatchMinimumResponseFine->GetValue() - KT_TOLERANCE)
        {
#ifdef KARTO_DEBUG2
          std::cout << " ACCEPTED" << std::endl;
#endif
          rMeans.push_back(mean);
          rCovariances.push_back(covariance);
          LinkChainToScan(chain, pScan, mean, covariance);
        }
        else
        {
#ifdef KARTO_DEBUG2
          std::cout << response << "(< " << m_pOpenMapper->m_pLinkMatchMinimumResponseFine->GetValue() << ") REJECTED" << std::endl;
#endif
        }
      }
    }
  }
  
  void MapperGraph::LinkChainToScan(const LocalizedLaserScanList& rChain, LocalizedLaserScan* pScan, const Pose2& rMean, const Matrix3& rCovariance)
  {
    Pose2 pose = pScan->GetReferencePose(m_pOpenMapper->m_pUseScanBarycenter->GetValue());

    LocalizedLaserScan* pClosestScan = GetClosestScanToPose(rChain, pose);
    assert(pClosestScan != nullptr);

    Pose2 closestScanPose = pClosestScan->GetReferencePose(m_pOpenMapper->m_pUseScanBarycenter->GetValue());

    kt_double squaredDistance = pose.GetPosition().SquaredDistance(closestScanPose.GetPosition());
    if (squaredDistance < math::Square(m_pOpenMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
    {
      LinkObjects(pClosestScan, pScan, rMean, rCovariance);
#ifdef KARTO_DEBUG2
      std::cout << "Linking scan " << pScan->GetStateId() << " to chain scan " << pClosestScan->GetStateId() << std::endl;
#endif
    }
  }
  
  std::vector<LocalizedLaserScanList> MapperGraph::FindNearChains(LocalizedLaserScan* pScan)
  {
    std::vector<LocalizedLaserScanList> nearChains;

    Pose2 scanPose = pScan->GetReferencePose(m_pOpenMapper->m_pUseScanBarycenter->GetValue());

    // to keep track of which scans have been added to a chain
    LocalizedLaserScanList processed;

    const LocalizedLaserScanList nearLinkedScans = FindNearLinkedScans(pScan, m_pOpenMapper->m_pLinkScanMaximumDistance->GetValue());
    for (const auto& pNearScan : nearLinkedScans)
    {
      if (pNearScan == pScan)
      {
        continue;
      }

      // scan has already been processed, skip
      if (std::find(processed.begin(), processed.end(), pNearScan) != processed.end())
      {
        continue;
      }
      
#ifdef KARTO_DEBUG2
      std::cout << "BUILDING CHAIN: Scan " << pScan->GetStateId() << " is near " << pNearScan->GetStateId() << " (< " << m_pOpenMapper->m_pLinkScanMaximumDistance->GetValue() << ")" << std::endl;
#endif
      
      processed.push_back(pNearScan);

      // build up chain
      kt_bool isValidChain = true;
      LocalizedLaserScanList chain;

      LocalizedLaserScanList scans = m_pOpenMapper->m_pMapperSensorManager->GetScans(pNearScan->GetSensorIdentifier());

      kt_int32s nearScanIndex = m_pOpenMapper->m_pMapperSensorManager->GetScanIndex(pNearScan);
      assert(nearScanIndex >= 0);

      // add scans before current scan being processed
      for (kt_int32s candidateScanIndex = nearScanIndex - 1; candidateScanIndex >= 0; candidateScanIndex--)
      {
        LocalizedLaserScan* pCandidateScan = scans[candidateScanIndex];

        // chain is invalid--contains scan being added
        if (pCandidateScan == pScan)
        {
#ifdef KARTO_DEBUG2
          std::cout << "INVALID CHAIN: Scan " << pScan->GetStateId() << " is not allowed in chain." << std::endl;
#endif
          isValidChain = false;
        }

        Pose2 candidatePose = pCandidateScan->GetReferencePose(m_pOpenMapper->m_pUseScanBarycenter->GetValue());
        kt_double squaredDistance = scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

        if (squaredDistance < math::Square(m_pOpenMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
        {
          chain.push_back(pCandidateScan);
          processed.push_back(pCandidateScan);

#ifdef KARTO_DEBUG2
          std::cout << "Building chain for " << pScan->GetStateId() << ": [ ";
          for (const auto& chainScan : chain)
          {
            std::cout << chainScan->GetStateId() << " ";
          }
          std::cout << "]" << std::endl;
#endif
        }
        else
        {
          break;
        }
      }

      chain.push_back(pNearScan);

      // add scans after current scan being processed
      kt_size_t end = scans.size();
      for (kt_size_t candidateScanIndex = nearScanIndex + 1; candidateScanIndex < end; candidateScanIndex++)
      {
        LocalizedLaserScan* pCandidateScan = scans[candidateScanIndex];

        if (pCandidateScan == pScan)
        {
#ifdef KARTO_DEBUG2
          std::cout << "INVALID CHAIN: Scan " << pScan->GetStateId() << " is not allowed in chain." << std::endl;
#endif
          isValidChain = false;
        }

        Pose2 candidatePose = pCandidateScan->GetReferencePose(m_pOpenMapper->m_pUseScanBarycenter->GetValue());;
        kt_double squaredDistance = scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

        if (squaredDistance < math::Square(m_pOpenMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
        {
          chain.push_back(pCandidateScan);
          processed.push_back(pCandidateScan);

#ifdef KARTO_DEBUG2
          std::cout << "Building chain for " << pScan->GetStateId() << ": [ ";
          for (const auto& chainScan : chain)
          {
            std::cout << chainScan->GetStateId() << " ";
          }
          std::cout << "]" << std::endl;
#endif
        }
        else
        {
          break;
        }
      }

      if (isValidChain)
      {
        // add chain to collection
        nearChains.push_back(chain);
      }
    }
    
    return nearChains;
  }
  
  LocalizedLaserScanList MapperGraph::FindNearLinkedScans(LocalizedLaserScan* pScan, kt_double maxDistance)
  {
    NearScanVisitor* pVisitor = new NearScanVisitor(pScan, maxDistance, m_pOpenMapper->m_pUseScanBarycenter->GetValue());
    LocalizedObjectList nearLinkedObjects = m_pTraversal->Traverse(GetVertex(pScan), pVisitor);
    delete pVisitor;
    
    LocalizedLaserScanList nearLinkedScans;
    for (const auto& pObject : nearLinkedObjects)
    {
      LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
      if (pScan != nullptr)
      {
        nearLinkedScans.push_back(pScan);
      }
    }
    
    return nearLinkedScans;
  }
  
  LocalizedLaserScanList MapperGraph::FindOverlappingScans(karto::LocalizedLaserScan *pScan)
  {
    LocalizedLaserScanList nearScans;

    const BoundingBox2& rBoundingBox = pScan->GetBoundingBox();
    
    const VertexList& vertices = GetVertices();
    for (const auto& pVertex : vertices)
    {
      LocalizedObject* pObject = pVertex->GetVertexObject();
      LocalizedLaserScan* pCandidateScan = dynamic_cast<LocalizedLaserScan*>(pObject);
      if (pCandidateScan == nullptr)
      {
        continue;
      }

      if (rBoundingBox.Intersects(pCandidateScan->GetBoundingBox()) == true)
      {
        nearScans.push_back(pCandidateScan);
      }
    }
    
    return nearScans;
  }

  
  Pose2 MapperGraph::ComputeWeightedMean(const Pose2List& rMeans, const std::vector<Matrix3>& rCovariances) const
  {
    assert(rMeans.size() == rCovariances.size());

    // compute sum of inverses and create inverse list
    std::vector<Matrix3> inverses;
    inverses.reserve(rCovariances.size());

    Matrix3 sumOfInverses;
    for (const auto& cov : rCovariances)
    {
      Matrix3 inverse = cov.Inverse();
      inverses.push_back(inverse);

      sumOfInverses += inverse;
    }
    Matrix3 inverseOfSumOfInverses = sumOfInverses.Inverse();

    // compute weighted mean
    Pose2 accumulatedPose;
    kt_double thetaX = 0.0;
    kt_double thetaY = 0.0;

    for (kt_size_t i = 0; i < inverses.size(); i++)
    {
      Pose2 pose = rMeans[i];
      kt_double angle = pose.GetHeading();
      thetaX += cos(angle);
      thetaY += sin(angle);

      Matrix3 weight = inverseOfSumOfInverses * inverses[i];
      accumulatedPose += weight * pose;
    }

    thetaX /= rMeans.size();
    thetaY /= rMeans.size();
    accumulatedPose.SetHeading(atan2(thetaY, thetaX));
    
    return accumulatedPose;
  }
  
  LocalizedLaserScanList MapperGraph::FindPossibleLoopClosure(LocalizedLaserScan* pScan, const Identifier& rSensorName, kt_int32u& rStartScanIndex)
  {
    LocalizedLaserScanList chain; // return value
    
    Pose2 pose = pScan->GetReferencePose(m_pOpenMapper->m_pUseScanBarycenter->GetValue());
    
    // possible loop closure chain should not include close scans that have a
    // path of links to the scan of interest
    const LocalizedLaserScanList nearLinkedScans = FindNearLinkedScans(pScan, m_pOpenMapper->m_pLoopSearchMaximumDistance->GetValue());
    
    LocalizedLaserScanList scans = m_pOpenMapper->m_pMapperSensorManager->GetScans(rSensorName);
    kt_size_t nScans = scans.size();
    for (; rStartScanIndex < nScans; rStartScanIndex++)
    {
      LocalizedLaserScan* pCandidateScan = scans[rStartScanIndex];

      Pose2 candidateScanPose = pCandidateScan->GetReferencePose(m_pOpenMapper->m_pUseScanBarycenter->GetValue());

      kt_double squaredDistance = candidateScanPose.GetPosition().SquaredDistance(pose.GetPosition());
      if (squaredDistance < math::Square(m_pOpenMapper->m_pLoopSearchMaximumDistance->GetValue()) + KT_TOLERANCE)
      {
        // a linked scan cannot be in the chain
        if (std::find(nearLinkedScans.begin(), nearLinkedScans.end(), pCandidateScan) != nearLinkedScans.end())
        {
          chain.clear();
        }
        else
        {
          chain.push_back(pCandidateScan);
        }
      }
      else
      {
        // return chain if it is long "enough"
        if (chain.size() >= m_pOpenMapper->m_pLoopMatchMinimumChainSize->GetValue())
        {
          return chain;
        }
        else
        {
          chain.clear();
        }
      }
    }
    
    return chain;
  }
  
  void MapperGraph::CorrectPoses()
  {
    // If no solver is set, we skip pose graph correction. Loop closure
    // constraints are still detected and added to the graph, but poses
    // are not optimized. This is acceptable for small operational areas.
    ScanSolver* pSolver = m_pOpenMapper->m_pScanSolver.get();
    if (pSolver != nullptr)
    {
      pSolver->Compute();
      
      for (const auto& correction : pSolver->GetCorrections())
      {
        LocalizedObject* pObject = m_pOpenMapper->m_pMapperSensorManager->GetLocalizedObject(correction.first);
        LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);

        if (pScan != nullptr)
        {
          pScan->SetSensorPose(correction.second);
        }
        else
        {
          pObject->SetCorrectedPose(correction.second);
        }
      }
      
      pSolver->Clear();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Default constructor
   */
  OpenMapper::OpenMapper(kt_bool multiThreaded)
    : Module("OpenMapper")
    , m_pScanSolver(nullptr)
    , m_Initialized(false)
    , m_MultiThreaded(multiThreaded)
    , m_pSequentialScanMatcher(nullptr)
    , m_pMapperSensorManager(nullptr)
    , m_pGraph(nullptr)
  {
    InitializeParameters();
  }

  /**
   * Default constructor
   */
  OpenMapper::OpenMapper(const char* pName, kt_bool multiThreaded)
    : Module(pName)
    , m_pScanSolver(nullptr)
    , m_Initialized(false)
    , m_MultiThreaded(multiThreaded)
    , m_pSequentialScanMatcher(nullptr)
    , m_pMapperSensorManager(nullptr)
    , m_pGraph(nullptr)
  {
    InitializeParameters();
  }

  /**
   * Destructor
   */
  OpenMapper::~OpenMapper()
  {
    Reset();

    delete m_pMapperSensorManager;
  }

  void OpenMapper::InitializeParameters()
  {
    m_pUseScanMatching = new Parameter<kt_bool>(GetParameterSet(), "UseScanMatching", "Mapper::Use::Scan Matching", "UseScanMatching", true);
    m_pUseScanBarycenter = new Parameter<kt_bool>(GetParameterSet(), "UseScanBarycenter", "Mapper::Use::Scan Barycenter", "UseScanBarycenter", true);
    m_pMinimumTravelDistance = new Parameter<kt_double>(GetParameterSet(), "MinimumTravelDistance", "Mapper::Minimum Travel::Distance", "MinimumTravelDistance", 0.2);
    m_pMinimumTravelHeading = new Parameter<kt_double>(GetParameterSet(), "MinimumTravelHeading", "Mapper::Minimum Travel::Heading", "MinimumTravelHeading", math::DegreesToRadians(20));

    m_pScanBufferSize = new Parameter<kt_int32u>(GetParameterSet(), "ScanBufferSize", "Mapper::Scan Buffer::Size", "ScanBufferSize", 70);
    m_pScanBufferMaximumScanDistance = new Parameter<kt_double>(GetParameterSet(), "ScanBufferMaximumScanDistance", "Mapper::Scan Buffer::Maximum Scan Distance", "ScanBufferMaximumScanDistance", 20);
    m_pUseResponseExpansion = new Parameter<kt_bool>(GetParameterSet(), "UseResponseExpansion", "Mapper::Use::Response Expansion", "UseResponseExpansion", false);

    m_pDistanceVariancePenalty = new Parameter<kt_double>(GetParameterSet(), "DistanceVariancePenalty", "Mapper::Scan Matcher::Distance Variance Penalty", "DistanceVariancePenalty",  math::Square(0.3));
    m_pMinimumDistancePenalty = new Parameter<kt_double>(GetParameterSet(), "MinimumDistancePenalty", "Mapper::Scan Matcher::Minimum Distance Penalty", "MinimumDistancePenalty", 0.5);
    m_pAngleVariancePenalty = new Parameter<kt_double>(GetParameterSet(), "AngleVariancePenalty", "Mapper::Scan Matcher::Angle Variance Penalty", "AngleVariancePenalty", math::Square(math::DegreesToRadians(20)));
    m_pMinimumAnglePenalty = new Parameter<kt_double>(GetParameterSet(), "MinimumAnglePenalty", "Mapper::Scan Matcher::Minimum Angle Penalty", "MinimumAnglePenalty", 0.9);
    
    m_pLinkMatchMinimumResponseFine = new Parameter<kt_double>(GetParameterSet(), "LinkMatchMinimumResponseFine", "Mapper::Link::Match Minimum Response Fine", "LinkMatchMinimumResponseFine", 0.6);
    m_pLinkScanMaximumDistance = new Parameter<kt_double>(GetParameterSet(), "LinkScanMaximumDistance", "Mapper::Link::Scan Maximum Distance", "LinkScanMaximumDistance", 5.0);

    m_pCorrelationSearchSpaceDimension = new Parameter<kt_double>(GetParameterSet(), "CorrelationSearchSpaceDimension", "Mapper::Correlation Search Space::Dimension", "CorrelationSearchSpaceDimension", 0.3);
    m_pCorrelationSearchSpaceResolution = new Parameter<kt_double>(GetParameterSet(), "CorrelationSearchSpaceResolution", "Mapper::Correlation Search Space::Resolution", "CorrelationSearchSpaceResolution", 0.01);
    m_pCorrelationSearchSpaceSmearDeviation = new Parameter<kt_double>(GetParameterSet(), "CorrelationSearchSpaceSmearDeviation", "Mapper::Correlation Search Space::Smear Deviation", "CorrelationSearchSpaceSmearDeviation", 0.03);
    m_pCoarseSearchAngleOffset = new Parameter<kt_double>(GetParameterSet(), "CoarseSearchAngleOffset", "Mapper::Scan Matcher::Coarse Search Angle Offset", "CoarseSearchAngleOffset", math::DegreesToRadians(20));
    m_pFineSearchAngleOffset = new Parameter<kt_double>(GetParameterSet(), "FineSearchAngleOffset", "Mapper::Scan Matcher::Fine Search Angle Offset", "FineSearchAngleOffset", math::DegreesToRadians(0.2));
    m_pCoarseAngleResolution = new Parameter<kt_double>(GetParameterSet(), "CoarseAngleResolution", "Mapper::Scan Matcher::Coarse Angle Resolution", "CoarseAngleResolution", math::DegreesToRadians(2));
    
    m_pLoopSearchSpaceDimension = new Parameter<kt_double>(GetParameterSet(), "LoopSearchSpaceDimension", "Mapper::Loop Correlation Search Space::Dimension", "LoopSearchSpaceDimension", 8.0);
    m_pLoopSearchSpaceResolution = new Parameter<kt_double>(GetParameterSet(), "LoopSearchSpaceResolution", "Mapper::Loop Correlation Search Space::Resolution", "LoopSearchSpaceResolution", 0.05);
    m_pLoopSearchSpaceSmearDeviation = new Parameter<kt_double>(GetParameterSet(), "LoopSearchSpaceSmearDeviation", "Mapper::Loop Correlation Search Space::Smear Deviation", "LoopSearchSpaceSmearDeviation", 0.03);

    m_pLoopSearchMaximumDistance = new Parameter<kt_double>(GetParameterSet(), "LoopSearchMaximumDistance", "Mapper::Loop::Search Maximum Distance", "LoopSearchMaximumDistance", 4.0);
    m_pLoopMatchMinimumChainSize = new Parameter<kt_int32u>(GetParameterSet(), "LoopMatchMinimumChainSize", "Mapper::Loop::Match::Minimum Chain Size", "LoopMatchMinimumChainSize", 10);
    m_pLoopMatchMaximumVarianceCoarse = new Parameter<kt_double>(GetParameterSet(), "LoopMatchMaximumVarianceCoarse", "Mapper::Loop::Match::Maximum Variance Coarse", "LoopMatchMaximumVarianceCoarse", math::Square(0.4));
    m_pLoopMatchMinimumResponseCoarse = new Parameter<kt_double>(GetParameterSet(), "LoopMatchMinimumResponseCoarse", "Mapper::Loop::Match::Minimum Response Coarse", "LoopMatchMinimumResponseCoarse", 0.7);
    m_pLoopMatchMinimumResponseFine = new Parameter<kt_double>(GetParameterSet(), "LoopMatchMinimumResponseFine", "Mapper::Loop::Match::Minimum Response Fine", "LoopMatchMinimumResponseFine", 0.7);
  }

  void OpenMapper::Initialize(kt_double rangeThreshold)
  {
    if (m_Initialized == false)
    {
      // create sequential scan and loop matcher
      m_pSequentialScanMatcher = ScanMatcher::Create(this, m_pCorrelationSearchSpaceDimension->GetValue(), m_pCorrelationSearchSpaceResolution->GetValue(), m_pCorrelationSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
      assert(m_pSequentialScanMatcher);

      m_pMapperSensorManager = new MapperSensorManager(m_pScanBufferSize->GetValue(), m_pScanBufferMaximumScanDistance->GetValue());

      m_pGraph = new MapperGraph(this, rangeThreshold);

      m_Initialized = true;
    }
    else
    {
      Log(LOG_WARNING, "Mapper already initialized");
    }
  }

  void OpenMapper::Reset()
  {
    Module::Reset();

    delete m_pSequentialScanMatcher;
    m_pSequentialScanMatcher = nullptr;

    delete m_pGraph;
    m_pGraph = nullptr;

    delete m_pMapperSensorManager;
    m_pMapperSensorManager = nullptr;

    m_Initialized = false;
  }

  kt_bool OpenMapper::Process(Object* pObject)
  {
    if (pObject == nullptr)
    {
      return false;
    }
    
    kt_bool isObjectProcessed = Module::Process(pObject);

    if (IsLaserRangeFinder(pObject))
    {
      LaserRangeFinder* pLaserRangeFinder = dynamic_cast<LaserRangeFinder*>(pObject);
 
      if (m_Initialized == false)
      {
        // initialize mapper with range threshold from sensor
        Initialize(pLaserRangeFinder->GetRangeThreshold());
      }
      
      // register sensor
      m_pMapperSensorManager->RegisterSensor(pLaserRangeFinder->GetIdentifier());
      
      return true;
    }
    
    LocalizedObject* pLocalizedObject = dynamic_cast<LocalizedObject*>(pObject);
    if (pLocalizedObject != nullptr)
    {
      LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
      if (pScan != nullptr)
      {
        karto::LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();
        
        // validate scan
        if (pLaserRangeFinder == nullptr)
        {
          return false;
        }
        
        // validate scan. Throws exception if scan is invalid.
        pLaserRangeFinder->Validate(pScan);
        
        if (m_Initialized == false)
        {
          // initialize mapper with range threshold from sensor
          Initialize(pLaserRangeFinder->GetRangeThreshold());
        }
      }

      // ensures sensor has been registered with mapper--does nothing if the sensor has already been registered
      m_pMapperSensorManager->RegisterSensor(pLocalizedObject->GetSensorIdentifier());

      // get last scan
      LocalizedLaserScan* pLastScan = m_pMapperSensorManager->GetLastScan(pLocalizedObject->GetSensorIdentifier());
      
      // update scans corrected pose based on last correction
      if (pLastScan != nullptr)
      {
        Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
        pLocalizedObject->SetCorrectedPose(lastTransform.TransformPose(pLocalizedObject->GetOdometricPose()));
      }
      
      // check custom data if object is not a scan or if scan has not moved enough (i.e.,
      // scan is outside minimum boundary or if heading is larger then minimum heading)
      if (pScan == nullptr || (!HasMovedEnough(pScan, pLastScan) && !pScan->IsGpsReadingValid()))
      {
        if (pLocalizedObject->HasCustomItem() == true)
        {
          m_pMapperSensorManager->AddLocalizedObject(pLocalizedObject);
          
          // add to graph
          m_pGraph->AddVertex(pLocalizedObject);
          m_pGraph->AddEdges(pLocalizedObject);
          
          return true;
        }
        
        return false;
      }
      
      /////////////////////////////////////////////
      // object is a scan
      
      Matrix3 covariance;
      covariance.SetToIdentity();
      
      // correct scan (if not first scan)
      if (m_pUseScanMatching->GetValue() && pLastScan != nullptr)
      {
        Pose2 bestPose;
        m_pSequentialScanMatcher->MatchScan(pScan, m_pMapperSensorManager->GetRunningScans(pScan->GetSensorIdentifier()), bestPose, covariance);
        pScan->SetSensorPose(bestPose);
      }
      
      ScanMatched(pScan);
      
      // add scan to buffer and assign id
      m_pMapperSensorManager->AddLocalizedObject(pLocalizedObject);
      
      if (m_pUseScanMatching->GetValue())
      {
        // add to graph
        m_pGraph->AddVertex(pScan);
        m_pGraph->AddEdges(pScan, covariance);
        
        m_pMapperSensorManager->AddRunningScan(pScan);
        
        std::vector<Identifier> sensorNames = m_pMapperSensorManager->GetSensorNames();
        for (const auto& sensorName : sensorNames)
        {
          m_pGraph->TryCloseLoop(pScan, sensorName);
        }      
      }
      
      m_pMapperSensorManager->SetLastScan(pScan);
      
      ScanMatchingEnd(pScan);
      
      isObjectProcessed = true;
    } // if object is LocalizedObject
    
    return isObjectProcessed;
  }

  kt_bool OpenMapper::HasMovedEnough(LocalizedLaserScan* pScan, LocalizedLaserScan* pLastScan) const
  {
    // test if first scan
    if (pLastScan == nullptr)
    {
      return true;
    }
    
    Pose2 lastScannerPose = pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
    Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

    // test if we have turned enough
    kt_double deltaHeading = math::NormalizeAngle(scannerPose.GetHeading() - lastScannerPose.GetHeading());
    if (fabs(deltaHeading) >= m_pMinimumTravelHeading->GetValue())
    {
      return true;
    }

    // test if we have moved enough
    kt_double squaredTravelDistance = lastScannerPose.GetPosition().SquaredDistance(scannerPose.GetPosition());
    if (squaredTravelDistance >= math::Square(m_pMinimumTravelDistance->GetValue()) - KT_TOLERANCE)
    {
      return true;
    }

    return false;
  }

  const LocalizedLaserScanList OpenMapper::GetAllProcessedScans() const
  {
    LocalizedLaserScanList allScans;

    if (m_pMapperSensorManager != nullptr)
    {
      allScans = m_pMapperSensorManager->GetAllScans();
    }

    return allScans;
  }

  const LocalizedObjectList OpenMapper::GetAllProcessedObjects() const
  {
    LocalizedObjectList allObjects;

    if (m_pMapperSensorManager != nullptr)
    {
      // BUGBUG: inefficient?  should return right away?
      allObjects = m_pMapperSensorManager->GetAllObjects();
    }

    return allObjects;
  }

  ScanSolver* OpenMapper::GetScanSolver() const
  {
    return m_pScanSolver.get();
  }

  void OpenMapper::SetScanSolver(ScanSolver* pScanOptimizer)
  {
    m_pScanSolver.reset(pScanOptimizer);
  }

  MapperGraph* OpenMapper::GetGraph() const
  {
    return m_pGraph;
  }

  ScanMatcher* OpenMapper::GetSequentialScanMatcher() const
  {
    return m_pSequentialScanMatcher;
  }

  ScanMatcher* OpenMapper::GetLoopScanMatcher() const
  {
    return m_pGraph->GetLoopScanMatcher();
  }  

}
