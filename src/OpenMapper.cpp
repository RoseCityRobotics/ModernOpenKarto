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
#include <sstream>

#include <stdexcept>
#include <queue>
#include <set>
#include <iterator>
#include <map>
#include <iostream>

#include <OpenMapper.h>

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
    SensorDataManager(uint32_t runningBufferMaximumSize, double runningBufferMaximumDistance)
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
    inline void AddObject(LocalizedObject* pObject, int32_t uniqueId)
    {
      // assign state id to object
      pObject->SetStateId(static_cast<uint32_t>(m_Objects.size()));

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
    inline int32_t GetScanIndex(LocalizedLaserScan* pScan)
    {
      int32_t targetStateId = pScan->GetStateId();
      auto it = std::lower_bound(m_Scans.begin(), m_Scans.end(), targetStateId,
        [](LocalizedLaserScan* a, int32_t stateId) {
          return a->GetStateId() < stateId;
        });
      if (it != m_Scans.end() && (*it)->GetStateId() == targetStateId)
      {
        return static_cast<int32_t>(std::distance(m_Scans.begin(), it));
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
      double squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
      while (m_RunningScans.size() > m_RunningBufferMaximumSize || squaredDistance > Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE)
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
    static int32_t ScanIndexComparator(LocalizedLaserScan* pScan1, LocalizedLaserScan* pScan2)
    {
      return pScan1->GetStateId() - pScan2->GetStateId();
    }
    
  private:
    LocalizedObjectList m_Objects;
    
    LocalizedLaserScanList m_Scans;
    LocalizedLaserScanList m_RunningScans;
    LocalizedLaserScan* m_pLastScan;

    uint32_t m_RunningBufferMaximumSize;
    double m_RunningBufferMaximumDistance;
  }; // SensorDataManager

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  using SensorDataManagerMap = std::map<std::string, SensorDataManager*>;

  struct MapperSensorManagerPrivate
  {
    // map from sensor name to sensor data
    SensorDataManagerMap m_SensorDataManagers;
    
    uint32_t m_RunningBufferMaximumSize;
    double m_RunningBufferMaximumDistance;
    
    int32_t m_NextUniqueId;    
    
    LocalizedObjectList m_Objects;
  };

  MapperSensorManager::MapperSensorManager(uint32_t runningBufferMaximumSize, double runningBufferMaximumDistance)
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
  
  
  void MapperSensorManager::RegisterSensor(const std::string& rSensorName)
  {
    if (GetSensorDataManager(rSensorName) == nullptr)
    {
      m_pMapperSensorManagerPrivate->m_SensorDataManagers[rSensorName] = new SensorDataManager(m_pMapperSensorManagerPrivate->m_RunningBufferMaximumSize, m_pMapperSensorManagerPrivate->m_RunningBufferMaximumDistance);
    }
  }

  LocalizedObject* MapperSensorManager::GetLocalizedObject(const std::string& rSensorName, int32_t stateId)
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
  LocalizedObject* MapperSensorManager::GetLocalizedObject(int32_t uniqueId)
  {
    assert(IsUpTo(uniqueId, (int32_t)m_pMapperSensorManagerPrivate->m_Objects.size()));
    return m_pMapperSensorManagerPrivate->m_Objects[uniqueId];
  }
  
  std::vector<std::string> MapperSensorManager::GetSensorNames()
  {
    std::vector<std::string> sensorNames;
    for (auto& entry : m_pMapperSensorManagerPrivate->m_SensorDataManagers)
    {
      sensorNames.push_back(entry.first);
    }

    return sensorNames;
  }
  
  LocalizedLaserScan* MapperSensorManager::GetLastScan(const std::string& rSensorName)
  {
    return GetSensorDataManager(rSensorName)->GetLastScan();
  }
  
  void MapperSensorManager::SetLastScan(LocalizedLaserScan* pScan)
  {
    GetSensorDataManager(pScan)->SetLastScan(pScan);
  }
  
  void MapperSensorManager::ClearLastScan(const std::string& rSensorName)
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
  
  LocalizedLaserScanList& MapperSensorManager::GetScans(const std::string& rSensorName)
  {
    return GetSensorDataManager(rSensorName)->GetScans();
  }
  
  int32_t MapperSensorManager::GetScanIndex(LocalizedLaserScan* pScan)
  {
    return GetSensorDataManager(pScan->GetSensorName())->GetScanIndex(pScan);
  }

  LocalizedLaserScanList& MapperSensorManager::GetRunningScans(const std::string& rSensorName)
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
  
  SensorDataManager* MapperSensorManager::GetSensorDataManager(const std::string& rSensorName)
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
    ScanMatcherGridSetBank(uint32_t nGrids, int32_t corrGridWidth, int32_t corrGridHeight,
                           double resolution, double smearDeviation,
                           int32_t searchSpaceGridWidth, int32_t searchSpaceGridHeight)
    {
      if (nGrids <= 0)
      {
        throw std::runtime_error("ScanMatcherGridSetBank requires at least 1 grid: " + ToString(nGrids));
      }
      for (uint32_t i = 0; i < nGrids; i++)
      {
        CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(corrGridWidth, corrGridHeight, resolution, smearDeviation);
        Grid<double>* pSearchSpaceProbs = Grid<double>::CreateGrid(searchSpaceGridWidth, searchSpaceGridHeight, resolution);
        GridIndexLookup<uint8_t>* pGridLookup = new GridIndexLookup<uint8_t>(pCorrelationGrid);
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
  
  ScanMatcher* ScanMatcher::Create(OpenMapper* pOpenMapper, double searchSize, double resolution, double smearDeviation, double rangeThreshold)
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
    
    assert(DoubleEqual(std::round(searchSize / resolution), (searchSize / resolution)));
    
    // calculate search space in grid coordinates
    uint32_t searchSpaceSideSize = static_cast<uint32_t>(std::round(searchSize / resolution) + 1);
    
    // compute requisite size of correlation grid (pad grid so that scan points can't fall off the grid
    // if a scan is on the border of the search space)
    uint32_t pointReadingMargin = static_cast<uint32_t>(ceil(rangeThreshold / resolution));
    
    int32_t gridSize = searchSpaceSideSize + 2 * pointReadingMargin;
    
    // create correlation grid
    assert(gridSize % 2 == 1);
    CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(gridSize, gridSize, resolution, smearDeviation);

    // create search space probabilities
    Grid<double>* pSearchSpaceProbs = Grid<double>::CreateGrid(searchSpaceSideSize, searchSpaceSideSize, resolution);
    
    GridIndexLookup<uint8_t>* pGridLookup = new GridIndexLookup<uint8_t>(pCorrelationGrid);
    
    ScanMatcher* pScanMatcher = new ScanMatcher(pOpenMapper);
    pScanMatcher->m_pScanMatcherGridSet = std::make_shared<ScanMatcherGridSet>(pCorrelationGrid, pSearchSpaceProbs, pGridLookup);
    
    if (pOpenMapper->IsMultiThreaded())
    {
      pScanMatcher->m_pScanMatcherGridSetBank = new ScanMatcherGridSetBank(10, gridSize, gridSize, resolution, smearDeviation, searchSpaceSideSize, searchSpaceSideSize);
    }

    return pScanMatcher;
  }
  
  double ScanMatcher::MatchScan(LocalizedLaserScan* pScan, const LocalizedLaserScanList& rBaseScans, Pose2& rMean, Matrix3& rCovariance, bool doPenalize, bool doRefineMatch)
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
    Grid<double>* pSearchSpaceProbs = pScanMatcherGridSet->m_pSearchSpaceProbs.get();
    
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
      rCovariance(2, 2) = 4 * Square(m_pOpenMapper->m_Config.coarseAngleResolution); // TH*TH
      
      if (m_pOpenMapper->IsMultiThreaded())
      {
        m_pScanMatcherGridSetBank->Return(pScanMatcherGridSet);
      }

      return 0.0;
    }
    
    // 2. get size of grid
    Rectangle2<int32_t> roi = pCorrelationGrid->GetROI();
    
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
    double bestResponse = CorrelateScan(pScanMatcherGridSet.get(), pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, m_pOpenMapper->m_Config.coarseSearchAngleOffset, m_pOpenMapper->m_Config.coarseAngleResolution, doPenalize, rMean, rCovariance, false);
    
    if (m_pOpenMapper->m_Config.useResponseExpansion == true)
    {
      if (DoubleEqual(bestResponse, 0.0))
      {
#ifdef KARTO_DEBUG
        std::cout << "Mapper Info: Expanding response search space!" << std::endl;
#endif
        // try and increase search angle offset with 20 degrees and do another match
        double newSearchAngleOffset = m_pOpenMapper->m_Config.coarseSearchAngleOffset;
        for (uint32_t i = 0; i < 3; i++)
        {
          newSearchAngleOffset += DegreesToRadians(20);
          
          bestResponse = CorrelateScan(pScanMatcherGridSet.get(), pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, newSearchAngleOffset, m_pOpenMapper->m_Config.coarseAngleResolution, doPenalize, rMean, rCovariance, false);
          
          if (DoubleEqual(bestResponse, 0.0) == false)
          {
            break;
          }
        }
        
#ifdef KARTO_DEBUG
        if (DoubleEqual(bestResponse, 0.0))
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
      bestResponse = CorrelateScan(pScanMatcherGridSet.get(), pScan, rMean, fineSearchOffset, fineSearchResolution, 0.5 * m_pOpenMapper->m_Config.coarseAngleResolution, m_pOpenMapper->m_Config.fineSearchAngleOffset, doPenalize, rMean, rCovariance, true);
    }
    
#ifdef KARTO_DEBUG
    std::cout << "  BEST POSE = " << rMean << " BEST RESPONSE = " << bestResponse << ",  VARIANCE = " << rCovariance(0, 0) << ", " << rCovariance(1, 1) << std::endl;
#endif
    
    assert(InRange(rMean.GetHeading(), -M_PI, M_PI));

    if (m_pOpenMapper->IsMultiThreaded())
    {
      m_pScanMatcherGridSetBank->Return(pScanMatcherGridSet);
    }

    return bestResponse;
  }
  
  
  double ScanMatcher::CorrelateScan(ScanMatcherGridSet* pScanMatcherGridSet, LocalizedLaserScan* pScan, const Pose2& rSearchCenter, const Vector2d& rSearchSpaceOffset, const Vector2d& rSearchSpaceResolution,
                                       double searchAngleOffset, double searchAngleResolution,	bool doPenalize, Pose2& rMean, Matrix3& rCovariance, bool doingFineMatch)
  {
    assert(searchAngleResolution != 0.0);

    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid.get();
    Grid<double>* pSearchSpaceProbs = pScanMatcherGridSet->m_pSearchSpaceProbs.get();
    GridIndexLookup<uint8_t>* pGridLookup = pScanMatcherGridSet->m_pGridLookup;

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
    
    uint32_t nX = static_cast<uint32_t>(std::round(rSearchSpaceOffset.GetX() * 2.0 / rSearchSpaceResolution.GetX()) + 1);
    std::vector<double> xPoses(nX), newPositionsX(nX), squaresX(nX);
    double startX = -rSearchSpaceOffset.GetX();
    for (uint32_t xIndex = 0; xIndex < nX; xIndex++)
    {
      double x = startX + xIndex * rSearchSpaceResolution.GetX();
      xPoses[xIndex] = x;
      newPositionsX[xIndex] = rSearchCenter.GetX() + x;
      squaresX[xIndex] = Square(x);
    }
    assert(DoubleEqual(xPoses.back(), -startX));
    
    uint32_t nY = static_cast<uint32_t>(std::round(rSearchSpaceOffset.GetY() * 2.0 / rSearchSpaceResolution.GetY()) + 1);
    std::vector<double> yPoses(nY), newPositionsY(nY), squaresY(nY);
    double startY = -rSearchSpaceOffset.GetY();
    for (uint32_t yIndex = 0; yIndex < nY; yIndex++)
    {
      double y = startY + yIndex * rSearchSpaceResolution.GetY();
      yPoses[yIndex] = y;
      newPositionsY[yIndex] = rSearchCenter.GetY() + y;
      squaresY[yIndex] = Square(y);
    }
    assert(DoubleEqual(yPoses.back(), -startY));
    
    // calculate pose response array size
    uint32_t nAngles = static_cast<uint32_t>(std::round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);
    std::vector<double> angles(nAngles);
    double angle = 0.0;
    double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
    for (uint32_t angleIndex = 0; angleIndex < nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * searchAngleResolution;
      angles[angleIndex] = angle;
    }
    assert(DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
    
    // allocate array
    uint32_t poseResponseSize = nX * nY * nAngles;
    std::vector<std::pair<double, Pose2> > poseResponses = std::vector<std::pair<double, Pose2> >(poseResponseSize);
    
    Vector2i startGridPoint = pCorrelationGrid->WorldToGrid(Vector2d(rSearchCenter.GetX() + startX, rSearchCenter.GetY() + startY));
    
    if (m_pOpenMapper->IsMultiThreaded())
    {
      unsigned int numThreads = std::thread::hardware_concurrency();
      if (numThreads == 0) numThreads = 4;

      std::vector<std::thread> threads;
      threads.reserve(numThreads);

      auto worker = [&](uint32_t yStart, uint32_t yEnd) {
        for (uint32_t yIndex = yStart; yIndex < yEnd; yIndex++)
        {
          double newPositionY = newPositionsY[yIndex];
          double squareY = squaresY[yIndex];
          for (uint32_t xIndex = 0; xIndex < nX; xIndex++)
          {
            double newPositionX = newPositionsX[xIndex];
            double squareX = squaresX[xIndex];
            Vector2i gridPoint = pCorrelationGrid->WorldToGrid(Vector2d(newPositionX, newPositionY));
            int32_t gridIndex = pCorrelationGrid->GridIndex(gridPoint);
            assert(gridIndex >= 0);
            double squaredDistance = squareX + squareY;
            for (uint32_t angleIndex = 0; angleIndex < nAngles; angleIndex++)
            {
              uint32_t poseResponseIndex = (nX * nAngles) * yIndex + nAngles * xIndex + angleIndex;
              double angle = angles[angleIndex];
              double response = GetResponse(pScanMatcherGridSet, angleIndex, gridIndex);
              if (doPenalize && (DoubleEqual(response, 0.0) == false))
              {
                double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance / m_pOpenMapper->m_Config.distanceVariancePenalty);
                distancePenalty = std::max(distancePenalty, m_pOpenMapper->m_Config.minimumDistancePenalty);
                double squaredAngleDistance = Square(angle - rSearchCenter.GetHeading());
                double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance / m_pOpenMapper->m_Config.angleVariancePenalty);
                anglePenalty = std::max(anglePenalty, m_pOpenMapper->m_Config.minimumAnglePenalty);
                response *= (distancePenalty * anglePenalty);
              }
              poseResponses[poseResponseIndex] = std::pair<double, Pose2>(response, Pose2(newPositionX, newPositionY, NormalizeAngle(angle)));
            }
          }
        }
      };

      uint32_t rowsPerThread = nY / numThreads;
      uint32_t remainder = nY % numThreads;
      uint32_t start = 0;
      for (unsigned int t = 0; t < numThreads; t++)
      {
        uint32_t end = start + rowsPerThread + (t < remainder ? 1 : 0);
        if (start < end)
          threads.emplace_back(worker, start, end);
        start = end;
      }
      for (auto& thread : threads)
        thread.join();
    }
    else
    {
      uint32_t poseResponseCounter = 0;
      for (uint32_t yIndex = 0; yIndex < nY; yIndex++)
      {
        double newPositionY = newPositionsY[yIndex];
        double squareY = squaresY[yIndex];

        for (uint32_t xIndex = 0; xIndex < nX; xIndex++)
        {
          double newPositionX = newPositionsX[xIndex];
          double squareX = squaresX[xIndex];

          Vector2i gridPoint = pCorrelationGrid->WorldToGrid(Vector2d(newPositionX, newPositionY));
          int32_t gridIndex = pCorrelationGrid->GridIndex(gridPoint);
          assert(gridIndex >= 0);

          double squaredDistance = squareX + squareY;
          for (uint32_t angleIndex = 0; angleIndex < nAngles; angleIndex++)
          {
            double angle = angles[angleIndex];

            double response = GetResponse(pScanMatcherGridSet, angleIndex, gridIndex);
            if (doPenalize && (DoubleEqual(response, 0.0) == false))
            {
              // simple model (approximate Gaussian) to take odometry into account

              double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance / m_pOpenMapper->m_Config.distanceVariancePenalty);
              distancePenalty = std::max(distancePenalty, m_pOpenMapper->m_Config.minimumDistancePenalty);

              double squaredAngleDistance = Square(angle - rSearchCenter.GetHeading());
              double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance / m_pOpenMapper->m_Config.angleVariancePenalty);
              anglePenalty = std::max(anglePenalty, m_pOpenMapper->m_Config.minimumAnglePenalty);

              response *= (distancePenalty * anglePenalty);
            }

            // store response and pose
            poseResponses[poseResponseCounter] = std::pair<double, Pose2>(response, Pose2(newPositionX, newPositionY, NormalizeAngle(angle)));
            poseResponseCounter++;
          }
        }
      }

      assert(poseResponseSize == poseResponseCounter);
    }
    
    // find value of best response (in [0; 1])
    double bestResponse = -1;
    for (uint32_t i = 0; i < poseResponseSize; i++)
    {
      bestResponse = std::max(bestResponse, poseResponses[i].first);
      
      // will compute positional covariance, save best relative probability for each cell
      if (!doingFineMatch)
      {
        const Pose2& rPose = poseResponses[i].second;
        Vector2i grid = pSearchSpaceProbs->WorldToGrid(rPose.GetPosition());
        
        double* ptr = (double*)pSearchSpaceProbs->GetDataPointer(grid);
        if (ptr == nullptr)
        {
          throw std::runtime_error("Mapper FATAL ERROR - Index out of range in probability search!");
        }
        
        *ptr = std::max(poseResponses[i].first, *ptr);
      }
    }
    
    // average all poses with same highest response
    Vector2d averagePosition;
    double thetaX = 0.0;
    double thetaY = 0.0;
    int32_t averagePoseCount = 0;
    for (uint32_t i = 0; i < poseResponseSize; i++)
    {
      if (DoubleEqual(poseResponses[i].first, bestResponse))
      {
        averagePosition += poseResponses[i].second.GetPosition();
        
        double heading = poseResponses[i].second.GetHeading();
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
      throw std::runtime_error("Mapper FATAL ERROR - Unable to find best position");
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
    
    assert(InRange(bestResponse, 0.0, 1.0));
    assert(InRange(rMean.GetHeading(), -M_PI, M_PI));
    
    return bestResponse;
  }
  
  void ScanMatcher::ComputePositionalCovariance(Grid<double>* pSearchSpaceProbs, const Pose2& rBestPose, double bestResponse,
                                                const Pose2& rSearchCenter, const Vector2d& rSearchSpaceOffset,
                                                const Vector2d& rSearchSpaceResolution, double searchAngleResolution, Matrix3& rCovariance)
  {
    // reset covariance to identity matrix
    rCovariance.SetToIdentity();
    
    // if best response is vary small return max variance
    if (bestResponse < KT_TOLERANCE)
    {
      rCovariance(0, 0) = MAX_VARIANCE; // XX
      rCovariance(1, 1) = MAX_VARIANCE; // YY
      rCovariance(2, 2) = 4 * Square(searchAngleResolution); // TH*TH
      
      return;
    }
    
    double accumulatedVarianceXX = 0;
    double accumulatedVarianceXY = 0;
    double accumulatedVarianceYY = 0;
    double norm = 0;
    
    double dx = rBestPose.GetX() - rSearchCenter.GetX();
    double dy = rBestPose.GetY() - rSearchCenter.GetY();
    
    double offsetX = rSearchSpaceOffset.GetX();
    double offsetY = rSearchSpaceOffset.GetY();
    
    uint32_t nX = static_cast<uint32_t>(std::round(offsetX * 2.0 / rSearchSpaceResolution.GetX()) + 1);
    double startX = -offsetX;
    assert(DoubleEqual(startX + (nX - 1) * rSearchSpaceResolution.GetX(), -startX));
    
    uint32_t nY = static_cast<uint32_t>(std::round(offsetY * 2.0 / rSearchSpaceResolution.GetY()) + 1);
    double startY = -offsetY;
    assert(DoubleEqual(startY + (nY - 1) * rSearchSpaceResolution.GetY(), -startY));
    
    for (uint32_t yIndex = 0; yIndex < nY; yIndex++)
    {
      double y = startY + yIndex * rSearchSpaceResolution.GetY();
      
      for (uint32_t xIndex = 0; xIndex < nX; xIndex++)
      {
        double x = startX + xIndex * rSearchSpaceResolution.GetX();
        
        Vector2i gridPoint = pSearchSpaceProbs->WorldToGrid(Vector2d(rSearchCenter.GetX() + x, rSearchCenter.GetY() + y));
        double response = *(pSearchSpaceProbs->GetDataPointer(gridPoint));
        
        // response is not a low response
        if (response >= (bestResponse - 0.1))
        {
          norm += response;
          accumulatedVarianceXX += (Square(x - dx) * response);
          accumulatedVarianceXY += ((x - dx) * (y - dy) * response);
          accumulatedVarianceYY += (Square(y - dy) * response);
        }
      }
    }
    
    if (norm > KT_TOLERANCE)
    {
      double varianceXX = accumulatedVarianceXX / norm;
      double varianceXY = accumulatedVarianceXY / norm;
      double varianceYY = accumulatedVarianceYY / norm;
      double varianceTHTH = 4 * Square(searchAngleResolution);
      
      // lower-bound variances so that they are not too small;
      // ensures that links are not too tight
      double minVarianceXX = 0.1 * Square(rSearchSpaceResolution.GetX());
      double minVarianceYY = 0.1 * Square(rSearchSpaceResolution.GetY());
      varianceXX = std::max(varianceXX, minVarianceXX);
      varianceYY = std::max(varianceYY, minVarianceYY);
      
      // increase variance for poorer responses
      double multiplier = 1.0 / bestResponse;
      rCovariance(0, 0) = varianceXX * multiplier;
      rCovariance(0, 1) = varianceXY * multiplier;
      rCovariance(1, 0) = varianceXY * multiplier;
      rCovariance(1, 1) = varianceYY * multiplier;
      rCovariance(2, 2) = varianceTHTH; // this value will be set in ComputeAngularCovariance
    }
    
    // if values are 0, set to MAX_VARIANCE
    // values might be 0 if points are too sparse and thus don't hit other points
    if (DoubleEqual(rCovariance(0, 0), 0.0))
    {
      rCovariance(0, 0) = MAX_VARIANCE;
    }
    
    if (DoubleEqual(rCovariance(1, 1), 0.0))
    {
      rCovariance(1, 1) = MAX_VARIANCE;
    }
  }
  
  void ScanMatcher::ComputeAngularCovariance(ScanMatcherGridSet* pScanMatcherGridSet, const Pose2& rBestPose, double bestResponse, const Pose2& rSearchCenter,
                                             double searchAngleOffset, double searchAngleResolution, Matrix3& rCovariance)
  {
    // NOTE: do not reset covariance matrix
    
    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid.get();
    
    // normalize angle difference
    double bestAngle = NormalizeAngleDifference(rBestPose.GetHeading(), rSearchCenter.GetHeading());
    
    Vector2i gridPoint = pCorrelationGrid->WorldToGrid(rBestPose.GetPosition());
    int32_t gridIndex = pCorrelationGrid->GridIndex(gridPoint);
    
    uint32_t nAngles = static_cast<uint32_t>(std::round(searchAngleOffset * 2 / searchAngleResolution) + 1);
    
    double angle = 0.0;
    double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
    
    double norm = 0.0;
    double accumulatedVarianceThTh = 0.0;
    for (uint32_t angleIndex = 0; angleIndex < nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * searchAngleResolution;
      double response = GetResponse(pScanMatcherGridSet, angleIndex, gridIndex);
      
      // response is not a low response
      if (response >= (bestResponse - 0.1))
      {
        norm += response;
        accumulatedVarianceThTh += (Square(angle - bestAngle) * response);
      }
    }
    assert(DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
    
    if (norm > KT_TOLERANCE)
    {
      if (accumulatedVarianceThTh < KT_TOLERANCE)
      {
        accumulatedVarianceThTh = Square(searchAngleResolution);
      }
      
      accumulatedVarianceThTh /= norm;
    }
    else
    {
      accumulatedVarianceThTh = 1000 * Square(searchAngleResolution);
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

    int32_t index = 0;
    size_t nScans = rScans.size();
    Vector2dList* pValidPoints = new Vector2dList[nScans];

    // first find all valid points
    for (const auto& pScan : rScans)
    {
      pValidPoints[index++] = FindValidPoints(pScan, rViewPoint);
    }

    // then add all valid points to correlation grid
    for (size_t i = 0; i < nScans; i++)
    {
      AddScanNew(pCorrelationGrid, pValidPoints[i]);
    }

    delete[] pValidPoints;
  }

  void ScanMatcher::AddScan(CorrelationGrid* pCorrelationGrid, LocalizedLaserScan* pScan, const Vector2d& rViewPoint, bool doSmear)
  {
    Vector2dList validPoints = FindValidPoints(pScan, rViewPoint);
    
    // put in all valid points
    for (const auto& point : validPoints)
    {
      Vector2i gridPoint = pCorrelationGrid->WorldToGrid(point);
      if (!IsUpTo(gridPoint.GetX(), pCorrelationGrid->GetROI().GetWidth()) || !IsUpTo(gridPoint.GetY(), pCorrelationGrid->GetROI().GetHeight()))
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

  void ScanMatcher::AddScanNew(CorrelationGrid* pCorrelationGrid, const Vector2dList& rValidPoints, bool doSmear)
  {
    // put in all valid points
    for (const auto& point : rValidPoints)
    {
      Vector2i gridPoint = pCorrelationGrid->WorldToGrid(point);
      if (!IsUpTo(gridPoint.GetX(), pCorrelationGrid->GetROI().GetWidth()) || !IsUpTo(gridPoint.GetY(), pCorrelationGrid->GetROI().GetHeight()))
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

    // For 360-degree lidars, the viewpoint occlusion test assumes a convex
    // semicircular point fan and rejects valid points at the wraparound boundary.
    // Skip the test entirely for full-rotation sensors.
    LaserRangeFinder* pLaser = pScan->GetLaserRangeFinder();
    if (pLaser != nullptr)
    {
      double angularRange = pLaser->GetMaximumAngle() - pLaser->GetMinimumAngle();
      if (angularRange >= (2.0 * M_PI) - KT_TOLERANCE)
      {
        return rPointReadings;
      }
    }

    // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
    const double minSquareDistance = Square(0.1); // in m^2

    // this iterator lags from the main iterator adding points only when the points are on
    // the same side as the viewpoint
    size_t trailingPointIndex = 0;
    Vector2dList validPoints;

    Vector2d firstPoint;
    bool firstTime = true;
    for (size_t i = 0; i < rPointReadings.size(); i++)
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
        double a = rViewPoint.GetY() - firstPoint.GetY();
        double b = firstPoint.GetX() - rViewPoint.GetX();
        double c = firstPoint.GetY() * rViewPoint.GetX() - firstPoint.GetX() * rViewPoint.GetY();
        double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;

        firstPoint = currentPoint;

        if (ss < 0.0)
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
  
  double ScanMatcher::GetResponse(ScanMatcherGridSet* pScanMatcherGridSet, uint32_t angleIndex, int32_t gridPositionIndex)
  {
    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid.get();
    GridIndexLookup<uint8_t>* pGridLookup = pScanMatcherGridSet->m_pGridLookup;
    
    double response = 0.0;
    
    // add up value for each point
    uint8_t* pByte = pCorrelationGrid->GetDataPointer() + gridPositionIndex;
    
    const LookupArray* pOffsets = pGridLookup->GetLookupArray(angleIndex);
    assert(pOffsets != nullptr);
    
    // get number of points in offset list
    uint32_t nPoints = pOffsets->GetSize();
    if (nPoints == 0)
    {
      return response;
    }
    
    // calculate response
    int32_t* pAngleIndexPointer = pOffsets->GetArrayPointer();
    for (uint32_t i = 0; i < nPoints; i++)
    {
      // ignore points that fall off the grid
      int32_t pointGridIndex = gridPositionIndex + pAngleIndexPointer[i];
      if (!IsUpTo(pointGridIndex, pCorrelationGrid->GetDataSize()))
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
      throw std::runtime_error("Correlation grid only available in single-threaded mode");
    }
    else
    {
      return m_pScanMatcherGridSet->m_pCorrelationGrid.get();
    }
  }
  
  Grid<double>* ScanMatcher::GetSearchGrid() const
  {
    if (m_pOpenMapper->IsMultiThreaded())
    {
      throw std::runtime_error("Search grid only available in single-threaded mode");
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
    NearScanVisitor(LocalizedLaserScan* pScan, double maxDistance, bool useScanBarycenter)
      : m_MaxDistanceSquared(Square(maxDistance))
      , m_UseScanBarycenter(useScanBarycenter)
    {
      m_CenterPose = pScan->GetReferencePose(m_UseScanBarycenter);
    }

    virtual bool Visit(Vertex<LocalizedObject*>* pVertex)
    {
      LocalizedObject* pObject = pVertex->GetVertexObject();

      LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
      
      // object is not a scan or wasn't scan matched, ignore
      if (pScan == nullptr)
      {
        return false;
      }
      
      Pose2 pose = pScan->GetReferencePose(m_UseScanBarycenter);

      double squaredDistance = pose.GetPosition().SquaredDistance(m_CenterPose.GetPosition());
      return (squaredDistance <= m_MaxDistanceSquared - KT_TOLERANCE);
    }

  protected:
    Pose2 m_CenterPose;
    double m_MaxDistanceSquared;
    bool m_UseScanBarycenter;

  }; // NearScanVisitor

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  MapperGraph::MapperGraph(OpenMapper* pOpenMapper, double rangeThreshold)
    : m_pOpenMapper(pOpenMapper)
  {
    m_pLoopScanMatcher = ScanMatcher::Create(pOpenMapper, m_pOpenMapper->m_Config.loopSearchSpaceDimension, m_pOpenMapper->m_Config.loopSearchSpaceResolution, m_pOpenMapper->m_Config.loopSearchSpaceSmearDeviation, rangeThreshold);
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
      const std::string& rSensorName = pObject->GetSensorName();
      
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
    
    const std::string& rSensorName = pScan->GetSensorName();
    
    Pose2List means;
    std::vector<Matrix3> covariances;

    LocalizedLaserScan* pLastScan = pSensorManager->GetLastScan(rSensorName);
    if (pLastScan == nullptr)
    {
      // first scan (link to first scan of other robots)

      assert(pSensorManager->GetScans(rSensorName).size() == 1);

      std::vector<std::string> sensorNames = pSensorManager->GetSensorNames();
      for (const auto& rCandidateSensorName : sensorNames)
      {
        // skip if candidate sensor is the same or other sensor has no scans
        if ((rCandidateSensorName == rSensorName) || (pSensorManager->GetScans(rCandidateSensorName).empty()))
        {
          continue;
        }

        Pose2 bestPose;
        Matrix3 covariance;
        double response = m_pOpenMapper->m_pSequentialScanMatcher->MatchScan(pScan, pSensorManager->GetScans(rCandidateSensorName), bestPose, covariance);
        LinkObjects(pSensorManager->GetScans(rCandidateSensorName)[0], pScan, bestPose, covariance);

        // only add to means and covariances if response was high "enough"
        if (response > m_pOpenMapper->m_Config.linkMatchMinimumResponseFine)
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
  
  bool MapperGraph::TryCloseLoop(LocalizedLaserScan* pScan, const std::string& rSensorName)
  {
    bool loopClosed = false;
    
    uint32_t scanIndex = 0;
    
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
      double coarseResponse = m_pLoopScanMatcher->MatchScan(pScan, candidateChain, bestPose, covariance, false, false);
      
      std::ostringstream message;
      message << "COARSE RESPONSE: " << coarseResponse << " (> " << m_pOpenMapper->m_Config.loopMatchMinimumResponseCoarse << ")\n";
      message << "            var: " << covariance(0, 0) << ",  " << covariance(1, 1) << " (< " << m_pOpenMapper->m_Config.loopMatchMaximumVarianceCoarse << ")";

      {
        std::string msg = message.str();
        for (auto& cb : m_pOpenMapper->m_MessageCallbacks) cb(msg);
      }
      
      if (((coarseResponse > m_pOpenMapper->m_Config.loopMatchMinimumResponseCoarse) &&
           (covariance(0, 0) < m_pOpenMapper->m_Config.loopMatchMaximumVarianceCoarse) &&
           (covariance(1, 1) < m_pOpenMapper->m_Config.loopMatchMaximumVarianceCoarse))
          ||
          // be more lenient if the variance is really small
          ((coarseResponse > 0.9 * m_pOpenMapper->m_Config.loopMatchMinimumResponseCoarse) &&
           (covariance(0, 0) < 0.01 * m_pOpenMapper->m_Config.loopMatchMaximumVarianceCoarse) &&
           (covariance(1, 1) < 0.01 * m_pOpenMapper->m_Config.loopMatchMaximumVarianceCoarse)))
      {
        // save for reversion
        Pose2 oldPose = pScan->GetSensorPose();
        
        pScan->SetSensorPose(bestPose);
        double fineResponse = m_pOpenMapper->m_pSequentialScanMatcher->MatchScan(pScan, candidateChain, bestPose, covariance, false);
        
        message.str("");
        message << "FINE RESPONSE: " << fineResponse << " (>" << m_pOpenMapper->m_Config.loopMatchMinimumResponseFine << ")";
        {
          std::string msg = message.str();
          for (auto& cb : m_pOpenMapper->m_MessageCallbacks) cb(msg);
        }
        
        if (fineResponse < m_pOpenMapper->m_Config.loopMatchMinimumResponseFine)
        {
          // failed verification test, revert
          pScan->SetSensorPose(oldPose);
          
          for (auto& cb : m_pOpenMapper->m_MessageCallbacks) cb("REJECTED!");
        }
        else
        {
          for (auto& cb : m_pOpenMapper->m_PreLoopClosedCallbacks) cb("Closing loop...");
          
          pScan->SetSensorPose(bestPose);
          LinkChainToScan(candidateChain, pScan, bestPose, covariance);
          CorrectPoses();

          for (auto& cb : m_pOpenMapper->m_PostLoopClosedCallbacks) cb("Loop closed!");
          for (auto& cb : m_pOpenMapper->m_ScansUpdatedCallbacks) cb();      

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
    double bestSquaredDistance = DBL_MAX;
    
    for (const auto& scan : rScans)
    {
      Pose2 scanPose = scan->GetReferencePose(m_pOpenMapper->m_Config.useScanBarycenter);

      double squaredDistance = rPose.GetPosition().SquaredDistance(scanPose.GetPosition());
      if (squaredDistance < bestSquaredDistance)
      {
        bestSquaredDistance = squaredDistance;
        pClosestScan = scan;
      }
    }
    
    return pClosestScan;
  }
    
  Edge<LocalizedObject*>* MapperGraph::AddEdge(LocalizedObject* pSourceObject, LocalizedObject* pTargetObject, bool& rIsNewEdge)
  {
    // check that vertex exists
    assert(pSourceObject->GetUniqueId() < (int32_t)m_Vertices.size());
    assert(pTargetObject->GetUniqueId() < (int32_t)m_Vertices.size());
    
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
    bool isNewEdge = true;
    Edge<LocalizedObject*>* pEdge = AddEdge(pFromObject, pToObject, isNewEdge);
    
    // only attach link information if the edge is new
    if (isNewEdge == true)
    {
      // Both poses must be in the same frame for the relative transform to be correct.
      // Use robot frame (GetCorrectedPose) consistently. rMean from MatchScan() is in
      // sensor frame, so convert it via GetCorrectedAt(). (slam_toolbox PR #362 fix)
      LocalizedLaserScan* pFromScan = dynamic_cast<LocalizedLaserScan*>(pFromObject);
      LocalizedLaserScan* pToScan = dynamic_cast<LocalizedLaserScan*>(pToObject);
      if (pFromScan != nullptr && pToScan != nullptr)
      {
        pEdge->SetLabel(new LinkInfo(pFromScan->GetCorrectedPose(), pToScan->GetCorrectedAt(rMean), rCovariance));
      }
      else if (pFromScan != nullptr)
      {
        pEdge->SetLabel(new LinkInfo(pFromScan->GetCorrectedPose(), rMean, rCovariance));
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
      int32_t chainCount = static_cast<int32_t>(nearChains.size());
      std::vector<bool> wasChainLinked(chainCount, false);
      Pose2List means(chainCount);
      std::vector<Matrix3> covariances(chainCount);

      unsigned int numThreads = std::thread::hardware_concurrency();
      if (numThreads == 0) numThreads = 4;

      std::vector<std::thread> threads;
      auto worker = [&](int32_t iStart, int32_t iEnd) {
        for (int32_t i = iStart; i < iEnd; i++)
        {
          const LocalizedLaserScanList& rChain = nearChains[i];
          if (rChain.size() < m_pOpenMapper->m_Config.loopMatchMinimumChainSize)
            continue;
          Pose2 mean;
          Matrix3 covariance;
          double response = m_pOpenMapper->GetSequentialScanMatcher()->MatchScan(pScan, rChain, mean, covariance, false);
          if (response > m_pOpenMapper->m_Config.linkMatchMinimumResponseFine - KT_TOLERANCE)
          {
            wasChainLinked[i] = true;
            means[i] = mean;
            covariances[i] = covariance;
          }
        }
      };

      int32_t rowsPerThread = chainCount / numThreads;
      int32_t remainder = chainCount % numThreads;
      int32_t start = 0;
      for (unsigned int t = 0; t < numThreads; t++)
      {
        int32_t end = start + rowsPerThread + (t < remainder ? 1 : 0);
        if (start < end)
          threads.emplace_back(worker, start, end);
        start = end;
      }
      for (auto& thread : threads)
        thread.join();

      for (int32_t i = 0; i < chainCount; i++)
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

        if (chain.size() < m_pOpenMapper->m_Config.loopMatchMinimumChainSize)
        {
#ifdef KARTO_DEBUG2
          std::cout << chain.size() << "(< " << m_pOpenMapper->m_Config.loopMatchMinimumChainSize << ") REJECTED" << std::endl;
#endif
          continue;
        }

        Pose2 mean;
        Matrix3 covariance;
        // match scan against "near" chain
        double response = m_pOpenMapper->m_pSequentialScanMatcher->MatchScan(pScan, chain, mean, covariance, false);
        if (response > m_pOpenMapper->m_Config.linkMatchMinimumResponseFine - KT_TOLERANCE)
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
          std::cout << response << "(< " << m_pOpenMapper->m_Config.linkMatchMinimumResponseFine << ") REJECTED" << std::endl;
#endif
        }
      }
    }
  }
  
  void MapperGraph::LinkChainToScan(const LocalizedLaserScanList& rChain, LocalizedLaserScan* pScan, const Pose2& rMean, const Matrix3& rCovariance)
  {
    Pose2 pose = pScan->GetReferencePose(m_pOpenMapper->m_Config.useScanBarycenter);

    LocalizedLaserScan* pClosestScan = GetClosestScanToPose(rChain, pose);
    assert(pClosestScan != nullptr);

    Pose2 closestScanPose = pClosestScan->GetReferencePose(m_pOpenMapper->m_Config.useScanBarycenter);

    double squaredDistance = pose.GetPosition().SquaredDistance(closestScanPose.GetPosition());
    if (squaredDistance < Square(m_pOpenMapper->m_Config.linkScanMaximumDistance) + KT_TOLERANCE)
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

    Pose2 scanPose = pScan->GetReferencePose(m_pOpenMapper->m_Config.useScanBarycenter);

    // to keep track of which scans have been added to a chain
    LocalizedLaserScanList processed;

    const LocalizedLaserScanList nearLinkedScans = FindNearLinkedScans(pScan, m_pOpenMapper->m_Config.linkScanMaximumDistance);
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
      std::cout << "BUILDING CHAIN: Scan " << pScan->GetStateId() << " is near " << pNearScan->GetStateId() << " (< " << m_pOpenMapper->m_Config.linkScanMaximumDistance << ")" << std::endl;
#endif
      
      processed.push_back(pNearScan);

      // build up chain
      bool isValidChain = true;
      LocalizedLaserScanList chain;

      LocalizedLaserScanList scans = m_pOpenMapper->m_pMapperSensorManager->GetScans(pNearScan->GetSensorName());

      int32_t nearScanIndex = m_pOpenMapper->m_pMapperSensorManager->GetScanIndex(pNearScan);
      assert(nearScanIndex >= 0);

      // add scans before current scan being processed
      for (int32_t candidateScanIndex = nearScanIndex - 1; candidateScanIndex >= 0; candidateScanIndex--)
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

        Pose2 candidatePose = pCandidateScan->GetReferencePose(m_pOpenMapper->m_Config.useScanBarycenter);
        double squaredDistance = scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

        if (squaredDistance < Square(m_pOpenMapper->m_Config.linkScanMaximumDistance) + KT_TOLERANCE)
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
      size_t end = scans.size();
      for (size_t candidateScanIndex = nearScanIndex + 1; candidateScanIndex < end; candidateScanIndex++)
      {
        LocalizedLaserScan* pCandidateScan = scans[candidateScanIndex];

        if (pCandidateScan == pScan)
        {
#ifdef KARTO_DEBUG2
          std::cout << "INVALID CHAIN: Scan " << pScan->GetStateId() << " is not allowed in chain." << std::endl;
#endif
          isValidChain = false;
        }

        Pose2 candidatePose = pCandidateScan->GetReferencePose(m_pOpenMapper->m_Config.useScanBarycenter);;
        double squaredDistance = scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

        if (squaredDistance < Square(m_pOpenMapper->m_Config.linkScanMaximumDistance) + KT_TOLERANCE)
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
  
  LocalizedLaserScanList MapperGraph::FindNearLinkedScans(LocalizedLaserScan* pScan, double maxDistance)
  {
    NearScanVisitor* pVisitor = new NearScanVisitor(pScan, maxDistance, m_pOpenMapper->m_Config.useScanBarycenter);
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
    double thetaX = 0.0;
    double thetaY = 0.0;

    for (size_t i = 0; i < inverses.size(); i++)
    {
      Pose2 pose = rMeans[i];
      double angle = pose.GetHeading();
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
  
  LocalizedLaserScanList MapperGraph::FindPossibleLoopClosure(LocalizedLaserScan* pScan, const std::string& rSensorName, uint32_t& rStartScanIndex)
  {
    LocalizedLaserScanList chain; // return value
    
    Pose2 pose = pScan->GetReferencePose(m_pOpenMapper->m_Config.useScanBarycenter);
    
    // possible loop closure chain should not include close scans that have a
    // path of links to the scan of interest
    const LocalizedLaserScanList nearLinkedScans = FindNearLinkedScans(pScan, m_pOpenMapper->m_Config.loopSearchMaximumDistance);
    
    LocalizedLaserScanList scans = m_pOpenMapper->m_pMapperSensorManager->GetScans(rSensorName);
    size_t nScans = scans.size();
    for (; rStartScanIndex < nScans; rStartScanIndex++)
    {
      LocalizedLaserScan* pCandidateScan = scans[rStartScanIndex];

      Pose2 candidateScanPose = pCandidateScan->GetReferencePose(m_pOpenMapper->m_Config.useScanBarycenter);

      double squaredDistance = candidateScanPose.GetPosition().SquaredDistance(pose.GetPosition());
      if (squaredDistance < Square(m_pOpenMapper->m_Config.loopSearchMaximumDistance) + KT_TOLERANCE)
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
        if (chain.size() >= m_pOpenMapper->m_Config.loopMatchMinimumChainSize)
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
          // Solver corrections are in robot frame, not sensor frame.
          // Use SetCorrectedPoseAndUpdate() to set the robot pose directly
          // and refresh point readings. (slam_toolbox PR #362 fix)
          pScan->SetCorrectedPoseAndUpdate(correction.second);
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
  OpenMapper::OpenMapper(bool multiThreaded)
    : m_Name("OpenMapper")
    , m_pScanSolver(nullptr)
    , m_Initialized(false)
    , m_MultiThreaded(multiThreaded)
    , m_pSequentialScanMatcher(nullptr)
    , m_pMapperSensorManager(nullptr)
    , m_pGraph(nullptr)
  {
  }

  /**
   * Constructor with name
   */
  OpenMapper::OpenMapper(const std::string& rName, bool multiThreaded)
    : m_Name(rName)
    , m_pScanSolver(nullptr)
    , m_Initialized(false)
    , m_MultiThreaded(multiThreaded)
    , m_pSequentialScanMatcher(nullptr)
    , m_pMapperSensorManager(nullptr)
    , m_pGraph(nullptr)
  {
  }

  /**
   * Destructor
   */
  OpenMapper::~OpenMapper()
  {
    Reset();

    delete m_pMapperSensorManager;
  }

  void OpenMapper::Initialize(double rangeThreshold)
  {
    if (m_Initialized == false)
    {
      // create sequential scan and loop matcher
      m_pSequentialScanMatcher = ScanMatcher::Create(this, m_Config.correlationSearchSpaceDimension, m_Config.correlationSearchSpaceResolution, m_Config.correlationSearchSpaceSmearDeviation, rangeThreshold);
      assert(m_pSequentialScanMatcher);

      m_pMapperSensorManager = new MapperSensorManager(m_Config.scanBufferSize, m_Config.scanBufferMaximumScanDistance);

      m_pGraph = new MapperGraph(this, rangeThreshold);

      m_Initialized = true;
    }
    else
    {
    }
  }

  void OpenMapper::Reset()
  {
    m_Sensors.clear();

    delete m_pSequentialScanMatcher;
    m_pSequentialScanMatcher = nullptr;

    delete m_pGraph;
    m_pGraph = nullptr;

    delete m_pMapperSensorManager;
    m_pMapperSensorManager = nullptr;

    m_Initialized = false;
  }

  bool OpenMapper::Process(LocalizedLaserScan* pScan)
  {
    if (pScan == nullptr)
    {
      return false;
    }

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
    m_pMapperSensorManager->RegisterSensor(pScan->GetSensorName());

    // get last scan
    LocalizedLaserScan* pLastScan = m_pMapperSensorManager->GetLastScan(pScan->GetSensorName());

    // update scans corrected pose based on last correction
    if (pLastScan != nullptr)
    {
      Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
      pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
    }

    // check if scan has not moved enough (i.e.,
    // scan is outside minimum boundary or if heading is larger then minimum heading)
    if (!HasMovedEnough(pScan, pLastScan) && !pScan->IsGpsReadingValid())
    {
      return false;
    }

    /////////////////////////////////////////////
    // object is a scan

    Matrix3 covariance;
    covariance.SetToIdentity();

    // correct scan (if not first scan)
    if (m_Config.useScanMatching && pLastScan != nullptr)
    {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan, m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()), bestPose, covariance);
      pScan->SetSensorPose(bestPose);
    }

    ScanMatched(pScan);

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddLocalizedObject(pScan);

    if (m_Config.useScanMatching)
    {
      // add to graph
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, covariance);

      m_pMapperSensorManager->AddRunningScan(pScan);

      std::vector<std::string> sensorNames = m_pMapperSensorManager->GetSensorNames();
      for (const auto& sensorName : sensorNames)
      {
        m_pGraph->TryCloseLoop(pScan, sensorName);
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    ScanMatchingEnd(pScan);

    return true;
  }

  bool OpenMapper::HasMovedEnough(LocalizedLaserScan* pScan, LocalizedLaserScan* pLastScan) const
  {
    // test if first scan
    if (pLastScan == nullptr)
    {
      return true;
    }
    
    Pose2 lastScannerPose = pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
    Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

    // test if we have turned enough
    double deltaHeading = NormalizeAngle(scannerPose.GetHeading() - lastScannerPose.GetHeading());
    if (fabs(deltaHeading) >= m_Config.minimumTravelHeading)
    {
      return true;
    }

    // test if we have moved enough
    double squaredTravelDistance = lastScannerPose.GetPosition().SquaredDistance(scannerPose.GetPosition());
    if (squaredTravelDistance >= Square(m_Config.minimumTravelDistance) - KT_TOLERANCE)
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
