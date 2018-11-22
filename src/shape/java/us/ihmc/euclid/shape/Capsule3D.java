package us.ihmc.euclid.shape;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.interfaces.Capsule3DBasics;
import us.ihmc.euclid.shape.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Capsule3D implements Capsule3DBasics, GeometryObject<Capsule3D>
{
   private final RigidBodyTransform pose = new RigidBodyTransform();
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   private double radius;
   private double length;

   private final Vector3DReadOnly axis = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return pose.getM02();
      }

      @Override
      public double getY()
      {
         return pose.getM12();
      }

      @Override
      public double getZ()
      {
         return pose.getM22();
      }
   };

   @Override
   public void set(Capsule3D other)
   {
      setPose(other);
      setRadius(other.getRadius());
      setLength(other.getLength());
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public void setLength(double length)
   {
      this.length = length;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getLength()
   {
      return length;
   }
   
   public Vector3DReadOnly getAxis()
   {
      return axis;
   }

   @Override
   public RigidBodyTransform getPose()
   {
      return pose;
   }

   @Override
   public RotationMatrix getOrientation()
   {
      return pose.getRotation();
   }

   @Override
   public Vector3DBasics getPosition()
   {
      return pose.getTranslation();
   }

   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return supplier;
   }

   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      this.supplier = newSupplier;
   }

   @Override
   public boolean epsilonEquals(Capsule3D other, double epsilon)
   {
      return Capsule3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(Capsule3D other, double epsilon)
   {
      return Capsule3DBasics.super.geometricallyEquals(other, epsilon);
   }
}
