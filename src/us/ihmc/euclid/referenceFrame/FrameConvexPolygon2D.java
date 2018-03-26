package us.ihmc.euclid.referenceFrame;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;

public class FrameConvexPolygon2D implements FrameConvexPolygon2DBasics
{
   

   @Override
   public FixedFramePoint2DBasics getVertexUnsafe(int index)
   {
      return null;
   }

   @Override
   public List<? extends FramePoint2DReadOnly> getUnmodifiableVertexBuffer()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FramePoint2DReadOnly getCentroid()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public boolean isClockwiseOrdered()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public boolean isUpToDate()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public int getNumberOfVertices()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public double getArea()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public BoundingBox2DReadOnly getBoundingBox()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public int getMinXIndex()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public int getMaxXIndex()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public int getMinYIndex()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public int getMaxYIndex()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public int getMinXMaxYIndex()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public int getMinXMinYIndex()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public int getMaxXMaxYIndex()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public int getMaxXMinYIndex()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void notifyVerticesChanged()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void clear()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void clearAndUpdate()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void update()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void updateBoundingBox()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void updateCentroidAndArea()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void addVertex(double x, double y)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void removeVertex(int indexOfVertexToRemove)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      // TODO Auto-generated method stub
      
   }

}
