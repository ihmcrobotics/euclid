package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class FrameShape3DPose implements FrameShape3DPoseBasics, GeometryObject<FrameShape3DPose>
{
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();

   private ReferenceFrame referenceFrame;
   private final FixedFrameRotationMatrixBasics shapeOrientation = EuclidFrameFactories.newLinkedFixedFrameRotationMatrixBasics(this, new RotationMatrix()
   {
      @Override
      public void setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
      {
         super.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         notifyChangeListeners();
      };

      @Override
      public void set(RotationMatrixReadOnly other)
      {
         super.set(other);
         notifyChangeListeners();
      };

      @Override
      public void transpose()
      {
         super.transpose();
         notifyChangeListeners();
      };
   });

   private final FixedFramePoint3DBasics shapePosition = EuclidFrameFactories.newLinkedFixedFramePoint3DBasics(this, new Point3D()
   {
      @Override
      public void setX(double x)
      {
         if (x != getX())
         {
            super.setX(x);
            notifyChangeListeners();
         }
      };

      @Override
      public void setY(double y)
      {
         if (y != getY())
         {
            super.setY(y);
            notifyChangeListeners();
         }
      };

      @Override
      public void setZ(double z)
      {
         if (z != getZ())
         {
            super.setZ(z);
            notifyChangeListeners();
         }
      };
   });

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

   public FrameShape3DPose()
   {
      this(ReferenceFrame.getWorldFrame());
   }

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

   public FrameShape3DPose(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      setIncludingFrame(referenceFrame, pose);
   }

   @Override
   public void set(FrameShape3DPose other)
   {
      FrameShape3DPoseBasics.super.set(other);
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

   public void notifyChangeListeners()
   {
      for (int i = 0; i < changeListeners.size(); i++)
         changeListeners.get(i).changed();
   }

   public void addChangeListeners(List<Shape3DChangeListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         addChangeListener(listeners.get(i));
      }
   }

   public void addChangeListener(Shape3DChangeListener listener)
   {
      changeListeners.add(listener);
   }

   public boolean removeChangeListener(Shape3DChangeListener listener)
   {
      return changeListeners.remove(listener);
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
