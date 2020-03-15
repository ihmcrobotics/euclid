package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameBoundingBox3D
      implements FrameBoundingBox3DBasics, EpsilonComparable<FrameBoundingBox3D>, Settable<FrameBoundingBox3D>, GeometricallyComparable<FrameBoundingBox3D>

{
   private ReferenceFrame referenceFrame;
   private final FixedFramePoint3DBasics minPoint = EuclidFrameFactories.newFixedFramePoint3DBasics(this);
   private final FixedFramePoint3DBasics maxPoint = EuclidFrameFactories.newFixedFramePoint3DBasics(this);

   public FrameBoundingBox3D()
   {
      setToNaN();
   }

   public FrameBoundingBox3D(ReferenceFrame referenceFrame)
   {
      setToNaN(referenceFrame);
   }

   public FrameBoundingBox3D(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      setIncludingFrame(min, max);
   }

   public FrameBoundingBox3D(Point3DReadOnly min, Point3DReadOnly max)
   {
      this(ReferenceFrame.getWorldFrame(), min, max);
   }

   public FrameBoundingBox3D(ReferenceFrame referenceFrame, Point3DReadOnly min, Point3DReadOnly max)
   {
      setIncludingFrame(referenceFrame, min, max);
   }

   public FrameBoundingBox3D(BoundingBox3DReadOnly boundingBox3DReadOnly)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), boundingBox3DReadOnly);
   }

   public FrameBoundingBox3D(ReferenceFrame referenceFrame, BoundingBox3DReadOnly boundingBox3DReadOnly)
   {
      setIncludingFrame(referenceFrame, boundingBox3DReadOnly);
   }

   public FrameBoundingBox3D(FrameBoundingBox3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   @Override
   public void set(FrameBoundingBox3D other)
   {
      FrameBoundingBox3DBasics.super.set(other);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public FixedFramePoint3DBasics getMinPoint()
   {
      return minPoint;
   }

   @Override
   public FixedFramePoint3DBasics getMaxPoint()
   {
      return maxPoint;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public boolean epsilonEquals(FrameBoundingBox3D other, double epsilon)
   {
      return FrameBoundingBox3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FrameBoundingBox3D other, double epsilon)
   {
      return FrameBoundingBox3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameBoundingBox3DReadOnly)
         return FrameBoundingBox3DBasics.super.equals((FrameBoundingBox3DReadOnly) object);
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
      return EuclidFrameIOTools.getFrameBoundingBox3DString(this);
   }
}
