package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

public class EuclidFrameShapeFactories
{
   private EuclidFrameShapeFactories()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   public static FixedFrameVector3DBasics newPositiveFixedFrameVector3DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return new FixedFrameVector3DBasics()
      {
         private double x, y, z;

         @Override
         public void setX(double x)
         {
            if (x < 0.0)
               throw new IllegalArgumentException("This vector's components cannot be negative: " + x);
            this.x = x;
         }

         @Override
         public void setY(double y)
         {
            if (y < 0.0)
               throw new IllegalArgumentException("This vector's components cannot be negative: " + y);
            this.y = y;
         }

         @Override
         public void setZ(double z)
         {
            if (z < 0.0)
               throw new IllegalArgumentException("This vector's components cannot be negative: " + z);
            this.z = z;
         }

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return y;
         }

         @Override
         public double getZ()
         {
            return z;
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameVector3DReadOnly)
               return FixedFrameVector3DBasics.super.equals((FrameVector3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            long bits = EuclidHashCodeTools.addToHashCode(EuclidHashCodeTools.toLongHashCode(x, y, z), getReferenceFrame());
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

   public static FixedFrameShape3DPoseBasics newFixedFrameShape3DPoseBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return new FixedFrameShape3DPoseBasics()
      {
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

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public FixedFramePoint3DBasics getShapePosition()
         {
            return shapePosition;
         }

         @Override
         public FixedFrameRotationMatrixBasics getShapeOrientation()
         {
            return shapeOrientation;
         }

         @Override
         public FrameVector3DReadOnly getXAxis()
         {
            return xAxis;
         }

         @Override
         public FrameVector3DReadOnly getYAxis()
         {
            return yAxis;
         }

         @Override
         public FrameVector3DReadOnly getZAxis()
         {
            return zAxis;
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
      };
   }
}
