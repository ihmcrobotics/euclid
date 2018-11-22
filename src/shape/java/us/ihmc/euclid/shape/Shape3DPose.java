package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Shape3DPose implements RigidBodyTransformBasics, GeometryObject<Shape3DPose>
{
   private final RotationMatrix shapeOrientation = new RotationMatrix();
   private final Point3D shapePosition = new Point3D();
   private final Vector3DReadOnly localVectorX = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return shapeOrientation.getM00();
      }

      @Override
      public double getY()
      {
         return shapeOrientation.getM10();
      }

      @Override
      public double getZ()
      {
         return shapeOrientation.getM20();
      }
   };

   private final Vector3DReadOnly localVectorY = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return shapeOrientation.getM01();
      }

      @Override
      public double getY()
      {
         return shapeOrientation.getM11();
      }

      @Override
      public double getZ()
      {
         return shapeOrientation.getM21();
      }
   };

   private final Vector3DReadOnly localVectorZ = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return shapeOrientation.getM02();
      }

      @Override
      public double getY()
      {
         return shapeOrientation.getM12();
      }

      @Override
      public double getZ()
      {
         return shapeOrientation.getM22();
      }
   };

   public Shape3DPose()
   {
      setToZero();
   }

   public Shape3DPose(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   public Shape3DPose(Pose3DReadOnly pose)
   {
      set(pose);
   }

   public void set(Pose3DReadOnly pose)
   {
      set(pose.getOrientation(), pose.getPosition());
   }

   @Override
   public void set(Shape3DPose other)
   {
      RigidBodyTransformBasics.super.set(other);
   }

   @Override
   public RotationMatrix getRotation()
   {
      return shapeOrientation;
   }

   @Override
   public Point3DBasics getTranslation()
   {
      return shapePosition;
   }

   public RotationMatrix getShapeOrientation()
   {
      return shapeOrientation;
   }

   public Point3D getShapePosition()
   {
      return shapePosition;
   }

   public Vector3DReadOnly getLocalVectorX()
   {
      return localVectorX;
   }

   public Vector3DReadOnly getLocalVectorY()
   {
      return localVectorY;
   }

   public Vector3DReadOnly getLocalVectorZ()
   {
      return localVectorZ;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(getTranslation());
      transform.transform(getRotation());
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getTranslation());
      transform.inverseTransform(getRotation());
   }

   @Override
   public boolean epsilonEquals(Shape3DPose other, double epsilon)
   {
      return shapePosition.epsilonEquals(other.getTranslation(), epsilon) && shapeOrientation.epsilonEquals(other.getRotation(), epsilon);
   }

   @Override
   public boolean geometricallyEquals(Shape3DPose other, double epsilon)
   {
      return shapePosition.geometricallyEquals(other.getTranslation(), epsilon) && shapeOrientation.geometricallyEquals(other.getRotation(), epsilon);
   }
}
