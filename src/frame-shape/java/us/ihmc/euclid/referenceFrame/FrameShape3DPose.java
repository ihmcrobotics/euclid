package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

public class FrameShape3DPose implements FrameShape3DPoseBasics, GeometryObject<FrameShape3DPose>
{
   private ReferenceFrame referenceFrame;
   private final FixedFrameRotationMatrixBasics shapeOrientation = EuclidFrameFactories.newFixedFrameRotationMatrixBasics(this);
   private final FixedFramePoint3DBasics shapePosition = EuclidFrameFactories.newFixedFramePoint3DBasics(this);

   /** Vector linked to the components of the x-axis unit-vector. */
   private final FrameVector3DReadOnly xAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(shapeOrientation::getM00,
                                                                                                   shapeOrientation::getM10,
                                                                                                   shapeOrientation::getM20,
                                                                                                   this);
   /** Vector linked to the components of the y-axis unit-vector. */
   private final FrameVector3DReadOnly yAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(shapeOrientation::getM01,
                                                                                                   shapeOrientation::getM11,
                                                                                                   shapeOrientation::getM21,
                                                                                                   this);
   /** Vector linked to the components of the z-axis unit-vector. */
   private final FrameVector3DReadOnly zAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(shapeOrientation::getM02,
                                                                                                   shapeOrientation::getM12,
                                                                                                   shapeOrientation::getM22,
                                                                                                   this);

   public FrameShape3DPose(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameShape3DPose(FramePose3DReadOnly pose)
   {
      setIncludingFrame(pose);
   }

   public FrameShape3DPose(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      setIncludingFrame(referenceFrame, pose);
   }

   @Override
   public void set(FrameShape3DPose other)
   {
      setIncludingFrame(other);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public FixedFrameRotationMatrixBasics getShapeOrientation()
   {
      return shapeOrientation;
   }

   @Override
   public FixedFramePoint3DBasics getShapePosition()
   {
      return shapePosition;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getXAxis()
   {
      return xAxis;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getYAxis()
   {
      return yAxis;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getZAxis()
   {
      return zAxis;
   }

   @Override
   public boolean epsilonEquals(FrameShape3DPose other, double epsilon)
   {
      return FrameShape3DPoseBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FrameShape3DPose other, double epsilon)
   {
      return FrameShape3DPoseBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameShape3DPoseReadOnly)
         return FrameShape3DPoseBasics.super.equals((FrameShape3DPoseReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(shapePosition, shapeOrientation);
   }

   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameShape3DPoseString(this);
   }
}
