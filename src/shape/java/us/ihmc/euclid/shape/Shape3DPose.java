package us.ihmc.euclid.shape;

import static us.ihmc.euclid.tools.EuclidCoreFactories.*;

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

   private final Vector3DReadOnly xAxis = newLinkedVector3DReadOnly(shapeOrientation::getM00, shapeOrientation::getM10, shapeOrientation::getM20);
   private final Vector3DReadOnly yAxis = newLinkedVector3DReadOnly(shapeOrientation::getM01, shapeOrientation::getM11, shapeOrientation::getM21);
   private final Vector3DReadOnly zAxis = newLinkedVector3DReadOnly(shapeOrientation::getM02, shapeOrientation::getM12, shapeOrientation::getM22);

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

   public Vector3DReadOnly getXAxis()
   {
      return xAxis;
   }

   public Vector3DReadOnly getYAxis()
   {
      return yAxis;
   }

   public Vector3DReadOnly getZAxis()
   {
      return zAxis;
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
