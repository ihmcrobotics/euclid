package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Class for holding the result of a collision query between two shapes.
 * 
 * @author Sylvain Bertrand
 */
public class EuclidShape3DCollisionResult
      implements EuclidShape3DCollisionResultBasics, EpsilonComparable<EuclidShape3DCollisionResult>, GeometricallyComparable<EuclidShape3DCollisionResult>, Settable<EuclidShape3DCollisionResult>
{
   /** Whether the shapes are colliding. */
   private boolean shapesAreColliding;
   /** The collision distance, either separation distance or penetration depth. */
   private double signedDistance;

   /** The first shape in the collision. */
   private Shape3DReadOnly shapeA;
   /** The second shape in the collision. */
   private Shape3DReadOnly shapeB;

   /** The key point on the shape A. */
   private final Point3D pointOnA = new Point3D();
   /** The surface normal at {@code pointOnA}. */
   private final Vector3D normalOnA = new Vector3D();

   /** The key point on the shape B. */
   private final Point3D pointOnB = new Point3D();
   /** The surface normal at {@code pointOnB}. */
   private final Vector3D normalOnB = new Vector3D();

   /**
    * Creates a new empty collision result.
    */
   public EuclidShape3DCollisionResult()
   {
   }

   /**
    * Clone constructor.
    * 
    * @param other the other object to clone. Not modified.
    */
   public EuclidShape3DCollisionResult(EuclidShape3DCollisionResultReadOnly other)
   {
      set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(EuclidShape3DCollisionResult other)
   {
      EuclidShape3DCollisionResultBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setShapesAreColliding(boolean shapesAreColliding)
   {
      this.shapesAreColliding = shapesAreColliding;
   }

   /** {@inheritDoc} */
   @Override
   public void setSignedDistance(double distance)
   {
      this.signedDistance = distance;
   }

   /** {@inheritDoc} */
   @Override
   public void setShapeA(Shape3DReadOnly shapeA)
   {
      this.shapeA = shapeA;
   }

   /** {@inheritDoc} */
   @Override
   public void setShapeB(Shape3DReadOnly shapeB)
   {
      this.shapeB = shapeB;
   }

   /** {@inheritDoc} */
   @Override
   public boolean areShapesColliding()
   {
      return shapesAreColliding;
   }

   /** {@inheritDoc} */
   @Override
   public double getSignedDistance()
   {
      return signedDistance;
   }

   /** {@inheritDoc} */
   @Override
   public Shape3DReadOnly getShapeA()
   {
      return shapeA;
   }

   /** {@inheritDoc} */
   @Override
   public Shape3DReadOnly getShapeB()
   {
      return shapeB;
   }

   /** {@inheritDoc} */
   @Override
   public Point3D getPointOnA()
   {
      return pointOnA;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3D getNormalOnA()
   {
      return normalOnA;
   }

   /** {@inheritDoc} */
   @Override
   public Point3D getPointOnB()
   {
      return pointOnB;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3D getNormalOnB()
   {
      return normalOnB;
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other   the other collision result to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two collision results are equal component-wise, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(EuclidShape3DCollisionResult other, double epsilon)
   {
      return EuclidShape3DCollisionResultBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests each feature of {@code this} against {@code other} for geometric similarity.
    * 
    * @param other   the other collision result to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each feature.
    * @return {@code true} if the two collision results are considered geometrically similar,
    *         {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(EuclidShape3DCollisionResult other, double epsilon)
   {
      return EuclidShape3DCollisionResultBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidShape3DCollisionResultReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof EuclidShape3DCollisionResultReadOnly)
         return EuclidShape3DCollisionResultBasics.super.equals((EuclidShape3DCollisionResultReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this collision result as follows:<br>
    * When shapes are colliding:
    * 
    * <pre>
    * Collision test result: colliding, depth: 0.539
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    * 
    * When shapes are not colliding:
    * 
    * <pre>
    * Collision test result: non-colliding, separating distance: 0.539
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    * 
    * @return the {@code String} representing this collisiont result.
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getEuclidShape3DCollisionResultString(this);
   }
}
