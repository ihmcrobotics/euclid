package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class FrameBoundingBox2D
      implements FrameBoundingBox2DBasics, EpsilonComparable<FrameBoundingBox2D>, Settable<FrameBoundingBox2D>, GeometricallyComparable<FrameBoundingBox2D>
{
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint2DBasics minPoint = EuclidFrameFactories.newFixedFramePoint2DBasics(this);
   private final FixedFramePoint2DBasics maxPoint = EuclidFrameFactories.newFixedFramePoint2DBasics(this);

   public FrameBoundingBox2D()
   {
      setToNaN();
   }

   public FrameBoundingBox2D(FramePoint2DReadOnly min, FramePoint2DReadOnly max)
   {
      setIncludingFrame(min, max);
   }

   public FrameBoundingBox2D(Point2DReadOnly min, Point2DReadOnly max)
   {
      this(ReferenceFrame.getWorldFrame(), min, max);
   }

   public FrameBoundingBox2D(ReferenceFrame referenceFrame, Point2DReadOnly min, Point2DReadOnly max)
   {
      setIncludingFrame(referenceFrame, min, max);
   }

   public FrameBoundingBox2D(BoundingBox2DReadOnly boundingBox2DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), boundingBox2DReadOnly);
   }

   public FrameBoundingBox2D(ReferenceFrame referenceFrame, BoundingBox2DReadOnly boundingBox2DReadOnly)
   {
      setIncludingFrame(referenceFrame, boundingBox2DReadOnly);
   }

   public FrameBoundingBox2D(FrameBoundingBox2DReadOnly other)
   {
      setIncludingFrame(other);
   }

   @Override
   public void set(FrameBoundingBox2D other)
   {
      FrameBoundingBox2DBasics.super.set(other);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public FixedFramePoint2DBasics getMinPoint()
   {
      return minPoint;
   }

   @Override
   public FixedFramePoint2DBasics getMaxPoint()
   {
      return maxPoint;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public boolean epsilonEquals(FrameBoundingBox2D other, double epsilon)
   {
      return FrameBoundingBox2DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FrameBoundingBox2D other, double epsilon)
   {
      return FrameBoundingBox2DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameBoundingBox2DReadOnly)
         return FrameBoundingBox2DBasics.super.equals((FrameBoundingBox2DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
   }

   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameBoundingBox2DString(this);
   }
}
