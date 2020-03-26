package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameSphere3DReadOnly extends Sphere3DReadOnly, FrameShape3DReadOnly
{
   @Override
   FramePoint3DReadOnly getPosition();

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
      return Sphere3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FramePoint3DBasics firstIntersectionToPack,
                                FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Sphere3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return Sphere3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, FramePoint3DBasics firstIntersectionToPack,
                                FramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Sphere3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWith(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                FixedFramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return Sphere3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      Sphere3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxSphere3D(this, destinationFrame, boundingBoxToPack);
   }

   @Override
   default FrameShape3DPoseReadOnly getPose()
   {
      return null;
   }

   @Override
   FrameSphere3DReadOnly copy();

   default boolean epsilonEquals(FrameSphere3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Sphere3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameSphere3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Sphere3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   default boolean equals(FrameSphere3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getPosition().equals(other.getPosition()) && getRadius() == other.getRadius();
   }
}
