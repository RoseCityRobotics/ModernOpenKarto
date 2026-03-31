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

#pragma once

#ifndef __OpenKarto_OccupancyGird_h__
#define __OpenKarto_OccupancyGird_h__

#include <Grid.h>
#include <SensorData.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Valid grid cell states
   */
  enum GridStates
  {
    GridStates_Unknown = 0,
    GridStates_Occupied = 100,
    GridStates_Free = 255
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class CellUpdater;
  class OpenMapper;

  /**
   * Occupancy grid definition. See GridStates for possible grid values.
   */
  class OccupancyGrid : public Grid<uint8_t>
  {

  private:
    friend class CellUpdater;
    friend class IncrementalOccupancyGrid;
    
  public:
    /**
     * Occupancy grid of given size
     * @param width width
     * @param height height
     * @param rOffset offset
     * @param resolution resolution
     */
    OccupancyGrid(int32_t width, int32_t height, const Vector2d& rOffset, double resolution);
    
  public:
    /**
     * Destructor
     */
    virtual ~OccupancyGrid();

  public:
    /**
     * Occupancy grid from the given scans using the given resolution
     * @note Please assign the returned occupancy grid to an OccupancyGridPtr to avoid memory leaks.
     * @param rScans list of scans
     * @param resolution resolution
     * @return occupancy grid from the given scans using the given resolution
     */
    [[nodiscard]] static OccupancyGrid* CreateFromScans(const LocalizedLaserScanList& rScans, double resolution);

    /**
     * Occupancy grid from the given scans using the given resolution
     * @note Please assign the returned occupancy grid to an OccupancyGridPtr to avoid memory leaks.
     * @param rScans scans
     * @param resolution resolution
     * @deprecated Please use CreateFromScans(const LocalizedLaserScanList& rScans, double resolution)
     * @warning Throws exception in Windows
     * @return occupancy grid from the given scans using the given resolution
     */
    [[deprecated, nodiscard]] static OccupancyGrid* CreateFromScans(const std::vector<std::shared_ptr<LocalizedRangeScan>>& rScans, double resolution);

    /**
     * Occupancy grid from the scans in the given mapper using the given resolution
     * @note Please assign the returned occupancy grid to an OccupancyGridPtr to avoid memory leaks.
     * @param pMapper mapper
     * @param resolution resolution
     * @return occupancy grid from the given scans using the given resolution
     */
    static OccupancyGrid* CreateFromMapper(OpenMapper* pMapper, double resolution);

  public:
    /**
     * Makes a clone
     * @return occupancy grid clone
     */
    OccupancyGrid* Clone() const;

    /** 
     * Checks if the given grid index is free
     * @param rGridIndex grid index
     * @return whether the cell at the given grid index is free space
     */
    inline bool IsFree(const Vector2i& rGridIndex) const
    {
      uint8_t* pOffsets = (uint8_t*)GetDataPointer(rGridIndex);
      return (*pOffsets == GridStates_Free);
    }
    
    /**
     * Casts a ray from the given point (up to the given max range)
     * and returns the distance to the closest obstacle
     * @param rPose2 starting point
     * @param maxRange maximum range
     * @return distance to closest obstacle
     */
    double RayCast(const Pose2& rPose2, double maxRange) const;

  protected:
    /**
     * Gets grid of cell hit counts
     * @return grid of cell hit counts
     */
    Grid<uint32_t>* GetCellHitsCounts()
    {
      return m_pCellHitsCnt.get();
    }

    /**
     * Get grid of cell pass counts
     * @return grid of cell pass counts
     */
    Grid<uint32_t>* GetCellPassCounts()
    {
      return m_pCellPassCnt.get();
    }
    
    /**
     * Calculates grid dimensions from localized laser scans and resolution
     * @param rScans scans
     * @param resolution resolution
     * @param rWidth (output parameter) width
     * @param rHeight (output parameter) height
     * @param rOffset (output parameter) offset
     */
    static void ComputeDimensions(const LocalizedLaserScanList& rScans, double resolution, int32_t& rWidth, int32_t& rHeight, Vector2d& rOffset);
    
    /**
     * Creates grid using scans 
     * @param rScans scans
     */
    virtual void CreateFromScans(const LocalizedLaserScanList& rScans);
        
    /**
     * Adds the scan's information to this grid's counters (optionally
     * update the grid's cells' occupancy status)
     * @param pScan laser scan
     * @param doUpdate whether to update the grid's cell's occupancy status
     * @return returns false if an endpoint fell off the grid, otherwise true
     */
    bool AddScan(LocalizedLaserScan* pScan, bool doUpdate = false);
    
    /**
     * Traces a beam from the start position to the end position marking
     * the bookkeeping arrays accordingly.
     * @param rWorldFrom start position of beam
     * @param rWorldTo end position of beam
     * @param isEndPointValid is the reading within the range threshold?
     * @param doUpdate whether to update the cells' occupancy status immediately
     * @return returns false if an endpoint fell off the grid, otherwise true
     */
    bool RayTrace(const Vector2d& rWorldFrom, const Vector2d& rWorldTo, bool isEndPointValid, bool doUpdate = false);
    
    /**
     * Updates a single cell's value based on the given counters
     * @param pCell cell
     * @param cellPassCnt cell pass count
     * @param cellHitCnt cell hit count
     */
    void UpdateCell(uint8_t* pCell, uint32_t cellPassCnt, uint32_t cellHitCnt);
    
    /**
     * Updates the grid based on the values in m_pCellHitsCnt and m_pCellPassCnt
     */
    void UpdateGrid();
    
    /**
     * Resizes the grid (deletes all old data)
     * @param width new width
     * @param height new height
     */
    virtual void Resize(int32_t width, int32_t height);
    
  protected:
    /**
     * Counters of number of times a beam passed through a cell
     */
    std::shared_ptr<Grid<uint32_t>> m_pCellPassCnt;

    /**
     * Counters of number of times a beam ended at a cell
     */
    std::shared_ptr<Grid<uint32_t>> m_pCellHitsCnt;

  private:
    CellUpdater* m_pCellUpdater;

    ////////////////////////////////////////////////////////////
    // NOTE: These two values are dependent on the resolution.  If the resolution is too small,
    // then not many beams will hit the cell!

    // Number of beams that must pass through a cell before it will be considered to be occupied
    // or unoccupied.  This prevents stray beams from messing up the map.
    uint32_t m_MinPassThrough;

    // Minimum ratio of beams hitting cell to beams passing through cell for cell to be marked as occupied
    double m_OccupancyThreshold;

  private:
    // restrict the following functions
    OccupancyGrid(const OccupancyGrid&);
    const OccupancyGrid& operator=(const OccupancyGrid&);
  }; // OccupancyGrid

  /**
   * Type declaration of OccupancyGrid managed by std::shared_ptr
   */
  using OccupancyGridPtr = std::shared_ptr<OccupancyGrid>;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Updates the cell at the given index based on the grid's hits and pass counters
   */    
  class CellUpdater : public Functor
  {
  public:
    /**
     * Cell updater for the given occupancy grid
     * @param pOccupancyGrid occupancy grid
     */
    CellUpdater(OccupancyGrid* pOccupancyGrid)
      : m_pOccupancyGrid(pOccupancyGrid)
    {
    }
    
    /**
     * Updates the cell at the given index based on the grid's hits and pass counters
     * @param index index
     */    
    virtual void operator() (uint32_t index);
    
  private:
    OccupancyGrid* m_pOccupancyGrid;
  }; // CellUpdater

  //@}

}


#endif // __OpenKarto_OccupancyGird_h__
