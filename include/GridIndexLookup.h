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

#ifndef __OpenKarto_GridIndexLookup_h__
#define __OpenKarto_GridIndexLookup_h__

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
   * Resizable array whose contents are not preseved upon resizing.
   * \todo Consider replacing with std::vector<int32_t>
   */
  class LookupArray
  {
  public:
    /**
     * Constructs empty array
     */
    LookupArray();

    /**
     * Destructor
     */
    virtual ~LookupArray();

  public:
    /**
     * Clears array
     */
    void Clear();

    /**
     * Gets size of this array
     * @return size of this array
     */
    uint32_t GetSize() const;

    /**
     * Sets size of this array (resize if not big enough)
     * @param size new size
     */
    void SetSize(uint32_t size);

    /**
     * Gets reference to value at given index
     * @param index index
     * @return reference to value at given index
     */
    inline int32_t& operator[](uint32_t index) 
    {
      assert(index < m_Size);

      return m_pArray[index]; 
    }

    /**
     * Gets value at given index
     * @param index index
     * @return value at given index
     */
    inline int32_t operator[](uint32_t index) const 
    {
      assert(index < m_Size);

      return m_pArray[index]; 
    }

    /**
     * Gets array pointer
     * @return array pointer
     */
    inline int32_t* GetArrayPointer()
    {
      return m_pArray;
    }

    /**
     * Gets array pointer (const version)
     * @return array pointer
     */
    inline int32_t* GetArrayPointer() const
    {
      return m_pArray;
    }

  private:
    int32_t* m_pArray;
    uint32_t m_Capacity;
    uint32_t m_Size;
  }; // LookupArray

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Create lookup tables for point readings at varying angles in grid.
   * For each angle, grid indexes are calculated for each point reading.
   * This is to speed up finding best angle/position for a localized laser scan
   * 
   * Used heavily in mapper and localizer. 
   * 
   * In the localizer, this is a huge speed up for calculating possible positions.  For each particle,
   * a probability is calculated.  The laser scan is the same, but all grid indexes at all possible angles are
   * calculated.  So when calculating the particle probability at a specific angle, the index table is used
   * to look up probabilities in the probability grid.
   */
  template<typename T>
  class GridIndexLookup
  {
  public:
    /**
     * Construct a GridIndexLookup with a grid
     * @param pGrid grid pointer
     */
    GridIndexLookup(Grid<T>* pGrid)
      : m_pGrid(pGrid)
      , m_Capacity(0)
      , m_Size(0)
      , m_ppLookupArray(nullptr)
    {
    }
    
    /**
     * Destructor
     */
    virtual ~GridIndexLookup()
    {
      DestroyArrays();
    }

  public:
    /**
     * Gets the lookup array for a particular angle index
     * @param index angle index
     * @return lookup array
     */
    const LookupArray* GetLookupArray(uint32_t index) const
    {
      assert(IsUpTo(index, m_Size));
      
      return m_ppLookupArray[index];
    }
    
    /**
     * Gets list of angles 
     * @return list of angles
     */
    const std::vector<double>& GetAngles() const
    {
      return m_Angles;
    }
    
    /**
     * Computes lookup table of the points of the given scan for the given angular space
     * @param pScan scan
     * @param angleCenter angle at center
     * @param angleOffset computes lookup arrays for the angles within this offset around angleCenter
     * @param angleResolution how fine a granularity to compute lookup arrays in the angular space
     */
    void ComputeOffsets(LocalizedLaserScan* pScan, double angleCenter, double angleOffset, double angleResolution)
    {
      assert(angleOffset != 0.0);
      assert(angleResolution != 0.0);
 
      uint32_t nAngles = static_cast<uint32_t>(std::round(angleOffset * 2.0 / angleResolution) + 1);
      SetSize(nAngles);

      //////////////////////////////////////////////////////
      // convert points into local coordinates of scan pose

      const Vector2dList& rPointReadings = pScan->GetPointReadings();

      // compute transform to scan pose
      Transform transform(pScan->GetSensorPose());

      Pose2List localPoints;
      for (const auto& point : rPointReadings)
      {
        // do inverse transform to get points in local coordinates
        Pose2 vec = transform.InverseTransformPose(Pose2(point, 0.0));
        localPoints.push_back(vec);
      }

      //////////////////////////////////////////////////////
      // create lookup array for different angles
      double angle = 0.0;
      double startAngle = angleCenter - angleOffset;
      for (uint32_t angleIndex = 0; angleIndex < nAngles; angleIndex++)
      {
        angle = startAngle + angleIndex * angleResolution;
        ComputeOffsets(angleIndex, angle, localPoints);
      }
      //assert(DoubleEqual(angle, angleCenter + angleOffset));
    }

  private:
    /**
     * Computes lookup value of points for given angle
     * @param angleIndex angle index
     * @param angle angle
     * @param rLocalPoints points in local coordinates
     */
    void ComputeOffsets(uint32_t angleIndex, double angle, const Pose2List& rLocalPoints)
    {
      m_ppLookupArray[angleIndex]->SetSize(static_cast<uint32_t>(rLocalPoints.size()));
      m_Angles[angleIndex] = angle;
      
      // set up point array by computing relative offsets to points readings
      // when rotated by given angle
      
      const Vector2d& rGridOffset = m_pGrid->GetCoordinateConverter()->GetOffset();
      
      double cosine = cos(angle);
      double sine = sin(angle);
      
      uint32_t readingIndex = 0;

      int32_t* pAngleIndexPointer = m_ppLookupArray[angleIndex]->GetArrayPointer();

      for (const auto& localPoint : rLocalPoints)
      {
        const Vector2d& rPosition = localPoint.GetPosition();
        
        // counterclockwise rotation and that rotation is about the origin (0, 0).
        Vector2d offset;
        offset.SetX(cosine * rPosition.GetX() -   sine * rPosition.GetY());
        offset.SetY(  sine * rPosition.GetX() + cosine * rPosition.GetY());
        
        // have to compensate for the grid offset when getting the grid index
        Vector2i gridPoint = m_pGrid->WorldToGrid(offset + rGridOffset);
        
        // use base GridIndex to ignore ROI 
        int32_t lookupIndex = m_pGrid->Grid<T>::GridIndex(gridPoint, false);

        pAngleIndexPointer[readingIndex] = lookupIndex;

        readingIndex++;
      }
      assert(readingIndex == rLocalPoints.size());
    }
        
    /**
     * Sets size of lookup table (resize if not big enough)
     * @param size new size
     */
    void SetSize(uint32_t size)
    {
      assert(size != 0);
      
      if (size > m_Capacity)
      {
        if (m_ppLookupArray != nullptr)
        {
          DestroyArrays();
        }
        
        m_Capacity = size;
        m_ppLookupArray = new LookupArray*[m_Capacity];
        for (uint32_t i = 0; i < m_Capacity; i++)
        {
          m_ppLookupArray[i] = new LookupArray();
        }        
      }
      
      m_Size = size;
      
      m_Angles.resize(size);
    }

    /**
     * Delete the arrays
     */
    void DestroyArrays()
    {
      for (uint32_t i = 0; i < m_Capacity; i++)
      {
        delete m_ppLookupArray[i];
      }
      
      delete[] m_ppLookupArray;
      m_ppLookupArray = nullptr;      
    }
    
  private:
    Grid<T>* m_pGrid; 

    uint32_t m_Capacity;
    uint32_t m_Size;
    
    LookupArray **m_ppLookupArray;

    // for sanity check
    std::vector<double> m_Angles;
  }; // class GridIndexLookup

  //@}

}

#endif // __OpenKarto_GridIndexLookup_h__
