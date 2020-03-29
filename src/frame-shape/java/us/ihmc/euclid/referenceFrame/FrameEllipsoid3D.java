package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameEllipsoid3D implements FrameEllipsoid3DBasics, GeometryObject<FrameEllipsoid3D>
{
   private ReferenceFrame referenceFrame;
   private final FixedFrameShape3DPose pose = new FixedFrameShape3DPose(this);
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();
   /**
    * Represents the radiusX, radiusY, and radiusZ of this ellipsoid.
    */
   private final FixedFrameVector3DBasics radii = EuclidFrameFactories.newLinkedFixedFrameVector3DBasics(this, new Vector3D()
   {
      @Override
      public void setX(double x)
      {
         if (x < 0.0)
            throw new IllegalArgumentException("The x-radius of an FrameEllipsoid3D cannot be negative: " + x);
         super.setX(x);
      }

      @Override
      public void setY(double y)
      {
         if (y < 0.0)
            throw new IllegalArgumentException("The y-radius of an FrameEllipsoid3D cannot be negative: " + y);
         super.setY(y);
      }

      @Override
      public void setZ(double z)
      {
         if (z < 0.0)
            throw new IllegalArgumentException("The z-radius of an FrameEllipsoid3D cannot be negative: " + z);
         super.setZ(z);
      }
   });

   public FrameEllipsoid3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameEllipsoid3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0, 1.0, 1.0);
   }

   public FrameEllipsoid3D(ReferenceFrame referenceFrame, double radiusX, double radiusY, double radiusZ)
   {
      setReferenceFrame(referenceFrame);
      getRadii().set(radiusX, radiusY, radiusZ);
   }

   public FrameEllipsoid3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY,
                           double radiusZ)
   {
      setIncludingFrame(referenceFrame, position, orientation, radiusX, radiusY, radiusZ);
   }

   public FrameEllipsoid3D(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(position, orientation, radiusX, radiusY, radiusZ);
   }

   public FrameEllipsoid3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(referenceFrame, pose, radiusX, radiusY, radiusZ);
   }

   public FrameEllipsoid3D(FramePose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(pose, radiusX, radiusY, radiusZ);
   }

   public FrameEllipsoid3D(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(referenceFrame, pose, radiusX, radiusY, radiusZ);
   }

   public FrameEllipsoid3D(FrameShape3DPoseReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(pose, radiusX, radiusY, radiusZ);
   }

   public FrameEllipsoid3D(ReferenceFrame referenceFrame, Ellipsoid3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   public FrameEllipsoid3D(FrameEllipsoid3DReadOnly other)
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
   public FixedFrameVector3DBasics getRadii()
   {
      return radii;
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
   public void set(FrameEllipsoid3D other)
   {
      FrameEllipsoid3DBasics.super.set(other);
   }

   @Override
   public FrameEllipsoid3D copy()
   {
      return new FrameEllipsoid3D(this);
   }

   @Override
   public boolean epsilonEquals(FrameEllipsoid3D other, double epsilon)
   {
      return FrameEllipsoid3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FrameEllipsoid3D other, double epsilon)
   {
      return FrameEllipsoid3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameEllipsoid3DReadOnly)
         return FrameEllipsoid3DBasics.super.equals((FrameEllipsoid3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(pose, radii);
   }

   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameEllipsoid3DString(this);
   }
}
