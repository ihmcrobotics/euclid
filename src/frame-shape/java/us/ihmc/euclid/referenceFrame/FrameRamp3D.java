package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameRamp3D implements FrameRamp3DBasics, GeometryObject<FrameRamp3D>
{
   private ReferenceFrame referenceFrame;
   private final FixedFrameShape3DPoseBasics pose = EuclidFrameShapeFactories.newFixedFrameShape3DPoseBasics(this);
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();
   /**
    * Represents the sizeX, sizeY, and sizeZ of this ramp.
    */
   private final FixedFrameVector3DBasics size = EuclidFrameShapeFactories.newPositiveFixedFrameVector3DBasics(this);
   
   public FrameRamp3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameRamp3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0, 1.0, 1.0);
   }

   public FrameRamp3D(ReferenceFrame referenceFrame, double sizeX, double sizeY, double sizeZ)
   {
      setReferenceFrame(referenceFrame);
      getSize().set(sizeX, sizeY, sizeZ);
   }

   public FrameRamp3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, position, orientation, sizeX, sizeY, sizeZ);
   }

   public FrameRamp3D(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(position, orientation, sizeX, sizeY, sizeZ);
   }

   public FrameRamp3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, pose, sizeX, sizeY, sizeZ);
   }

   public FrameRamp3D(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose, sizeX, sizeY, sizeZ);
   }

   public FrameRamp3D(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, pose, sizeX, sizeY, sizeZ);
   }

   public FrameRamp3D(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose, sizeX, sizeY, sizeZ);
   }

   public FrameRamp3D(ReferenceFrame referenceFrame, Ramp3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   public FrameRamp3D(FrameRamp3DReadOnly other)
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

   /** {@inheritDoc} */
   @Override
   public FixedFrameShape3DPoseBasics getPose()
   {
      return pose;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getSize()
   {
      return size;
   }

   /** {@inheritDoc} */
   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return supplier;
   }

   /** {@inheritDoc} */
   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      supplier = newSupplier;
   }

   @Override
   public void set(FrameRamp3D other)
   {
      FrameRamp3DBasics.super.set(other);
   }

   @Override
   public boolean epsilonEquals(FrameRamp3D other, double epsilon)
   {
      return FrameRamp3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FrameRamp3D other, double epsilon)
   {
      return FrameRamp3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameRamp3DReadOnly)
         return FrameRamp3DBasics.super.equals((FrameRamp3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(pose, size);
   }

   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameRamp3DString(this);
   }
}
