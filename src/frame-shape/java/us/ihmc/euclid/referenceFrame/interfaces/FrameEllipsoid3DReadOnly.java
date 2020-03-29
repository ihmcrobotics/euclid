package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameEllipsoid3DReadOnly extends Ellipsoid3DReadOnly, FrameShape3DReadOnly
{
   @Override
   FrameVector3DReadOnly getRadii();

   @Override
   FrameShape3DPoseReadOnly getPose();

   @Override
   default FrameRotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   @Override
   default FramePoint3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this ellipsoid.
    * </p>
    */
   @Override
   default FramePoint3DReadOnly getCentroid()
   {
      return getPosition();
   }

   default int intersectionWith(FrameLine3DReadOnly line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(FrameLine3DReadOnly line, FramePoint3DBasics firstIntersectionToPack, FramePoint3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(Line3DReadOnly line, FramePoint3DBasics firstIntersectionToPack, FramePoint3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(Line3DReadOnly line, FixedFramePoint3DBasics firstIntersectionToPack, FixedFramePoint3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(FrameLine3DReadOnly line, FixedFramePoint3DBasics firstIntersectionToPack, FixedFramePoint3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return Ellipsoid3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FramePoint3DBasics firstIntersectionToPack,
                                FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Ellipsoid3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return Ellipsoid3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, FramePoint3DBasics firstIntersectionToPack,
                                FramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Ellipsoid3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                FixedFramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return Ellipsoid3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxEllipsoid3D(this, destinationFrame, boundingBoxToPack);
   }

   @Override
   FixedFrameEllipsoid3DBasics copy();

   default boolean epsilonEquals(FrameEllipsoid3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Ellipsoid3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameEllipsoid3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Ellipsoid3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   default boolean equals(FrameEllipsoid3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getPose().equals(other.getPose()) && getRadii().equals(other.getRadii());
   }
}
