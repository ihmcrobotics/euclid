package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class FixedFrameShape3DPose implements FixedFrameShape3DPoseBasics, GeometryObject<FixedFrameShape3DPose>
{
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();

   private final ReferenceFrameHolder referenceFrameHolder;
   private final FixedFrameRotationMatrixBasics shapeOrientation;
   private final FixedFramePoint3DBasics shapePosition;

   /** Vector linked to the components of the x-axis unit-vector. */
   private final FrameVector3DReadOnly xAxis;
   /** Vector linked to the components of the y-axis unit-vector. */
   private final FrameVector3DReadOnly yAxis;
   /** Vector linked to the components of the z-axis unit-vector. */
   private final FrameVector3DReadOnly zAxis;

   public FixedFrameShape3DPose(ReferenceFrameHolder referenceFrameHolder)
   {
      this.referenceFrameHolder = referenceFrameHolder;

      shapeOrientation = EuclidFrameFactories.newLinkedFixedFrameRotationMatrixBasics(referenceFrameHolder, new RotationMatrix()
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

      shapePosition = EuclidFrameFactories.newLinkedFixedFramePoint3DBasics(this, new Point3D()
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

      xAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(shapeOrientation::getM00, shapeOrientation::getM10, shapeOrientation::getM20, this);
      yAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(shapeOrientation::getM01, shapeOrientation::getM11, shapeOrientation::getM21, this);
      zAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(shapeOrientation::getM02, shapeOrientation::getM12, shapeOrientation::getM22, this);
   }

   public FixedFrameShape3DPose(ReferenceFrameHolder referenceFrameHolder, Shape3DPoseReadOnly pose)
   {
      this(referenceFrameHolder);
      set(pose);
   }

   public FixedFrameShape3DPose(ReferenceFrameHolder referenceFrameHolder, RigidBodyTransformReadOnly pose)
   {
      this(referenceFrameHolder);
      set(pose);
   }

   @Override
   public void set(FixedFrameShape3DPose other)
   {
      FixedFrameShape3DPoseBasics.super.set(other);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrameHolder.getReferenceFrame();
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
   public boolean epsilonEquals(FixedFrameShape3DPose other, double epsilon)
   {
      return FixedFrameShape3DPoseBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FixedFrameShape3DPose other, double epsilon)
   {
      return FixedFrameShape3DPoseBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameShape3DPoseReadOnly)
         return FixedFrameShape3DPoseBasics.super.equals((FrameShape3DPoseReadOnly) object);
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
