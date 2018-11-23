package us.ihmc.euclid.shape;

import static us.ihmc.euclid.tools.EuclidCoreFactories.*;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.interfaces.Shape3DPoseBasics;
import us.ihmc.euclid.shape.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Shape3DPose implements Shape3DPoseBasics, GeometryObject<Shape3DPose>
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

   @Override
   public void set(Shape3DPose other)
   {
      Shape3DPoseBasics.super.set(other);
   }

   @Override
   public RotationMatrix getShapeOrientation()
   {
      return shapeOrientation;
   }

   @Override
   public Point3D getShapePosition()
   {
      return shapePosition;
   }

   @Override
   public Vector3DReadOnly getXAxis()
   {
      return xAxis;
   }

   @Override
   public Vector3DReadOnly getYAxis()
   {
      return yAxis;
   }

   @Override
   public Vector3DReadOnly getZAxis()
   {
      return zAxis;
   }

   @Override
   public boolean epsilonEquals(Shape3DPose other, double epsilon)
   {
      return Shape3DPoseBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(Shape3DPose other, double epsilon)
   {
      return Shape3DPoseBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Shape3DPoseReadOnly)
         return Shape3DPoseBasics.super.equals((Shape3DPoseReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      long hashCode = getShapePosition().hashCode();
      hashCode = EuclidHashCodeTools.combineHashCode(hashCode, getShapeOrientation().hashCode());
      return EuclidHashCodeTools.toIntHashCode(hashCode);
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getShape3DPoseString(this);
   }
}
