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

#include <memory>
#include <OccupancyGrid.h>
#include <OpenMapper.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  OccupancyGrid::OccupancyGrid(int32_t width, int32_t height, const Vector2d& rOffset, double resolution) 
    : Grid<uint8_t>(width, height)
    , m_pCellPassCnt(std::shared_ptr<Grid<uint32_t>>(Grid<uint32_t>::CreateGrid(0, 0, resolution)))
    , m_pCellHitsCnt(std::shared_ptr<Grid<uint32_t>>(Grid<uint32_t>::CreateGrid(0, 0, resolution)))
    , m_pCellUpdater(nullptr)
  {
    m_pCellUpdater = new CellUpdater(this);

    if (karto::DoubleEqual(resolution, 0.0))
    {
      throw std::runtime_error("Resolution cannot be 0");
    }

    m_MinPassThrough = 2;
    m_OccupancyThreshold = 0.1;

    GetCoordinateConverter()->SetScale(1.0 / resolution);
    GetCoordinateConverter()->SetOffset(rOffset);
  }

  OccupancyGrid::~OccupancyGrid()
  {
    delete m_pCellUpdater;
  }

  OccupancyGrid* OccupancyGrid::Clone() const
  {
    OccupancyGrid* pOccupancyGrid = new OccupancyGrid(GetWidth(), GetHeight(), GetCoordinateConverter()->GetOffset(), 1.0 / GetCoordinateConverter()->GetScale());
    memcpy(pOccupancyGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

    pOccupancyGrid->GetCoordinateConverter()->SetSize(GetCoordinateConverter()->GetSize());
    pOccupancyGrid->m_pCellPassCnt.reset(m_pCellPassCnt->Clone());
    pOccupancyGrid->m_pCellHitsCnt.reset(m_pCellHitsCnt->Clone());

    return pOccupancyGrid;
  }

  // KARTO_DEPRECATED
#ifdef WIN32
  OccupancyGrid* OccupancyGrid::CreateFromScans(const std::vector<std::shared_ptr<LocalizedRangeScan>>& /*rScans*/, double /*resolution*/)
  {
    throw std::runtime_error("OccupancyGrid::CreateFromScans - Not supported in Windows. Please Use CreateFromScans(const LocalizedLaserScanList& rScans, double resolution).");
  }
#else
  OccupancyGrid* OccupancyGrid::CreateFromScans(const std::vector<std::shared_ptr<LocalizedRangeScan>>& rScans, double resolution)
  {
    LocalizedLaserScanList scans;
    for (const auto& scan : rScans)
    {
      scans.push_back(scan.get());
    }

    return CreateFromScans(scans, resolution);
  }
#endif

  OccupancyGrid* OccupancyGrid::CreateFromScans(const LocalizedLaserScanList& rScans, double resolution)
  {
    if (rScans.size() == 0)
    {
      return nullptr;
    }

    int32_t width, height;
    Vector2d offset;
    OccupancyGrid::ComputeDimensions(rScans, resolution, width, height, offset);
    OccupancyGrid* pOccupancyGrid = new OccupancyGrid(width, height, offset, resolution);
    pOccupancyGrid->CreateFromScans(rScans);

    return pOccupancyGrid;      
  }

  OccupancyGrid* OccupancyGrid::CreateFromMapper(OpenMapper* pMapper, double resolution)
  {
    return karto::OccupancyGrid::CreateFromScans(pMapper->GetAllProcessedScans(), resolution);
  }

  void OccupancyGrid::ComputeDimensions(const LocalizedLaserScanList& rScans, double resolution, int32_t& rWidth, int32_t& rHeight, Vector2d& rOffset)
  {
    BoundingBox2 boundingBox;
    for (const auto& pLocalizedLaserScan : rScans)
    {
      if (pLocalizedLaserScan != nullptr)
      {
        boundingBox.Add(pLocalizedLaserScan->GetBoundingBox());
      }
    }

    double scale = 1.0 / resolution;
    Size2<double> size = boundingBox.GetSize();

    rWidth = static_cast<int32_t>(std::round(size.GetWidth() * scale));
    rHeight = static_cast<int32_t>(std::round(size.GetHeight() * scale));
    rOffset = boundingBox.GetMinimum();
  }

  void OccupancyGrid::CreateFromScans(const LocalizedLaserScanList& rScans)
  {
    m_pCellPassCnt->Resize(GetWidth(), GetHeight());
    m_pCellPassCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    m_pCellHitsCnt->Resize(GetWidth(), GetHeight());
    m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    for (const auto& pScan : rScans)
    {
      AddScan(pScan);
    }

    UpdateGrid();
  }

  bool OccupancyGrid::AddScan(LocalizedLaserScan* pScan, bool doUpdate)
  {
    double rangeThreshold = pScan->GetLaserRangeFinder()->GetRangeThreshold();
    double maxRange = pScan->GetLaserRangeFinder()->GetMaximumRange();
    double minRange = pScan->GetLaserRangeFinder()->GetMinimumRange();

    Vector2d scanPosition = pScan->GetSensorPose().GetPosition();

    // get scan point readings
    const Vector2dList& rPointReadings = pScan->GetPointReadings(false);

    bool scanInGrid = false;

    // draw lines from scan position to all point readings 
    for (const auto& pointReading : rPointReadings)
    {
      Vector2d point = pointReading;
      double range = scanPosition.Distance(point);
      bool isEndPointValid = range < (rangeThreshold - KT_TOLERANCE);

      if (range >= maxRange || range < minRange)
      {
        // ignore max range or min range readings
        continue;
      }
      else if (range >= rangeThreshold)
      {
        // trace up to range reading
        double ratio = rangeThreshold / range;
        double dx = point.GetX() - scanPosition.GetX();
        double dy = point.GetY() - scanPosition.GetY();
        point.SetX(scanPosition.GetX() + ratio * dx);
        point.SetY(scanPosition.GetY() + ratio * dy);
      }

      if (RayTrace(scanPosition, point, isEndPointValid, doUpdate))
      {
        scanInGrid = true;
      }
    }

    return scanInGrid;
  }

  double OccupancyGrid::RayCast(const Pose2& rPose2, double maxRange) const
  {
    double scale = GetCoordinateConverter()->GetScale();

    double x = rPose2.GetX();
    double y = rPose2.GetY();
    double theta = rPose2.GetHeading();

    double sinTheta = sin(theta);
    double cosTheta = cos(theta);

    double xStop = x + maxRange * cosTheta;
    double xSteps = 1 + fabs(xStop - x) * scale;

    double yStop = y + maxRange * sinTheta;
    double ySteps = 1 + fabs(yStop - y) * scale;

    double steps = std::max(xSteps, ySteps);
    double delta = maxRange / steps;
    double distance = delta;

    for (uint32_t i = 1; i < steps; i++)
    {
      double x1 = x + distance * cosTheta;
      double y1 = y + distance * sinTheta;

      Vector2i gridIndex = WorldToGrid(Vector2d(x1, y1));
      if (IsValidGridIndex(gridIndex) && IsFree(gridIndex))
      {
        distance = (i + 1) * delta;
      }
      else
      {
        break;
      }
    }

    return (distance < maxRange) ? distance : maxRange;
  }

  bool OccupancyGrid::RayTrace(const Vector2d& rWorldFrom, const Vector2d& rWorldTo, bool isEndPointValid, bool doUpdate)
  {
    assert(m_pCellPassCnt != nullptr && m_pCellHitsCnt != nullptr);

    Vector2i gridFrom = m_pCellPassCnt->WorldToGrid(rWorldFrom);
    Vector2i gridTo = m_pCellPassCnt->WorldToGrid(rWorldTo);

    CellUpdater* pCellUpdater = doUpdate ? m_pCellUpdater : nullptr;
    m_pCellPassCnt->TraceLine(gridFrom.GetX(), gridFrom.GetY(), gridTo.GetX(), gridTo.GetY(), pCellUpdater);        

    // for the end point
    if (isEndPointValid)
    {
      if (m_pCellPassCnt->IsValidGridIndex(gridTo))
      {
        int32_t index = m_pCellPassCnt->GridIndex(gridTo, false);

        uint32_t* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
        uint32_t* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

        // increment cell pass through and hit count
        pCellPassCntPtr[index]++;
        pCellHitCntPtr[index]++;

        if (doUpdate)
        {
          (*m_pCellUpdater)(index);
        }
      }
    }      

    return m_pCellPassCnt->IsValidGridIndex(gridTo);
  }

  void OccupancyGrid::UpdateCell(uint8_t* pCell, uint32_t cellPassCnt, uint32_t cellHitCnt)
  {
    if (cellPassCnt > m_MinPassThrough)
    {
      double hitRatio = static_cast<double>(cellHitCnt) / static_cast<double>(cellPassCnt);

      if (hitRatio > m_OccupancyThreshold)
      {
        *pCell = GridStates_Occupied;
      }
      else
      {
        *pCell = GridStates_Free;
      }
    }      
  }

  void OccupancyGrid::UpdateGrid()
  {
    assert(m_pCellPassCnt != nullptr && m_pCellHitsCnt != nullptr);

    // clear grid
    Clear();

    // set occupancy status of cells
    uint8_t* pDataPtr = GetDataPointer();
    uint32_t* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
    uint32_t* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

    uint32_t nBytes = GetDataSize();
    for (uint32_t i = 0; i < nBytes; i++, pDataPtr++, pCellPassCntPtr++, pCellHitCntPtr++)
    {
      UpdateCell(pDataPtr, *pCellPassCntPtr, *pCellHitCntPtr);
    }
  }

  void OccupancyGrid::Resize(int32_t width, int32_t height)
  {
    Grid<uint8_t>::Resize(width, height);

    m_pCellPassCnt->Resize(width, height);
    m_pCellHitsCnt->Resize(width, height);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  void CellUpdater::operator() (uint32_t index)
  {
    uint8_t* pDataPtr = m_pOccupancyGrid->GetDataPointer();
    uint32_t* pCellPassCntPtr = m_pOccupancyGrid->m_pCellPassCnt->GetDataPointer();
    uint32_t* pCellHitCntPtr = m_pOccupancyGrid->m_pCellHitsCnt->GetDataPointer();

    m_pOccupancyGrid->UpdateCell(&pDataPtr[index], pCellPassCntPtr[index], pCellHitCntPtr[index]);
  }

}