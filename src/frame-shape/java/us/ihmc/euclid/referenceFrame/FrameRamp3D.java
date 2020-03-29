package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameRamp3D implements FrameRamp3DBasics, GeometryObject<FrameRamp3D>
{
   private ReferenceFrame referenceFrame;
   private final FixedFrameShape3DPose pose = new FixedFrameShape3DPose(this);
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();
   /** Represents the sizeX, sizeY, and sizeZ of this ramp. */
   private final FixedFrameVector3DBasics size = EuclidFrameFactories.newLinkedFixedFrameVector3DBasics(this, new Vector3D()
   {
      @Override
      public void setX(double x)
      {
         if (x != getX())
         {
            if (x < 0.0)
               throw new IllegalArgumentException("The x-size of a FrameRamp3D cannot be negative: " + x);
            super.setX(x);
            notifyChangeListeners();
         }
      }

      @Override
      public void setY(double y)
      {
         if (y != getY())
         {
            if (y < 0.0)
               throw new IllegalArgumentException("The y-size of a FrameRamp3D cannot be negative: " + y);
            super.setY(y);
            notifyChangeListeners();
         }
      }

      @Override
      public void setZ(double z)
      {
         if (z != getZ())
         {
            if (z < 0.0)
               throw new IllegalArgumentException("The z-size of a FrameRamp3D cannot be negative: " + z);
            super.setZ(z);
            notifyChangeListeners();
         }
      }
   });

   private boolean rampFeaturesDirty = true;

   /** Length of the slope face of this ramp. */
   private double rampLength;
   /**
    * Positive angle in [0, <i>pi</i>] representing the angle formed by the bottom face and the slope
    * face.
    */
   private double angleOfRampIncline;

   private boolean centroidDirty = true;

   private final FixedFramePoint3DBasics centroid = EuclidFrameFactories.newLinkedFixedFramePoint3DBasics(this, new Point3D()
   {
      @Override
      public double getX()
      {
         updateCentroid();
         return super.getX();
      };

      @Override
      public double getY()
      {
         updateCentroid();
         return super.getY();
      };

      @Override
      public double getZ()
      {
         updateCentroid();
         return super.getZ();
      };
   });

   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();

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
      setupListeners();
   }

   public FrameRamp3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, position, orientation, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   public FrameRamp3D(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(position, orientation, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   public FrameRamp3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   public FrameRamp3D(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   public FrameRamp3D(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   public FrameRamp3D(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   public FrameRamp3D(ReferenceFrame referenceFrame, Ramp3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
      setupListeners();
   }

   public FrameRamp3D(FrameRamp3DReadOnly other)
   {
      setIncludingFrame(other);
      setupListeners();
   }

   private void setupListeners()
   {
      changeListeners.add(() -> rampFeaturesDirty = true);
      changeListeners.add(() -> centroidDirty = true);
      pose.addChangeListeners(changeListeners);
   }

   private void updateRamp()
   {
      if (!rampFeaturesDirty)
         return;

      rampLength = EuclidShapeTools.computeRamp3DLength(size.getX(), size.getZ());
      angleOfRampIncline = EuclidShapeTools.computeRamp3DIncline(size.getX(), size.getZ());
      rampFeaturesDirty = false;
   }

   private void updateCentroid()
   {
      if (!centroidDirty)
         return;

      EuclidShapeTools.computeRamp3DCentroid(pose, size, centroid);
      centroidDirty = false;
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

   @Override
   public FramePoint3DReadOnly getCentroid()
   {
      return centroid;
   }

   public void notifyChangeListeners()
   {
      for (int i = 0; i < changeListeners.size(); i++)
      {
         changeListeners.get(i).changed();
      }
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

   /**
    * Gets the length of this ramp's slope part.
    * <p>
    * Note that this is different than {@link #getSizeX()}. The returned value is equal to:
    * &radic;(this.length<sup>2</sup> + this.height<sup>2</sup>)
    * </p>
    *
    * @return the length of the slope.
    */
   @Override
   public double getRampLength()
   {
      updateRamp();
      return rampLength;
   }

   /**
    * Gets the angle formed by the slope and the bottom face.
    * <p>
    * The angle is positive and in [0, <i>pi</i>].
    * </p>
    *
    * @return the slope angle.
    */
   @Override
   public double getRampIncline()
   {
      updateRamp();
      return angleOfRampIncline;
   }

   @Override
   public FrameRamp3D copy()
   {
      return new FrameRamp3D(this);
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
