package us.ihmc.euclid.referenceFrame.collision;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultBasics;
import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;

/**
 * Class for holding the result of a collision query between two shapes and their respective
 * reference frame information.
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameShape3DCollisionResult implements EuclidFrameShape3DCollisionResultBasics

{
   /** Whether the shapes are colliding. */
   private boolean shapesAreColliding;
   /** The collision distance, either separation distance or penetration depth. */
   private double signedDistance;

   /** The first shape in the collision. */
   private FrameShape3DReadOnly shapeA;
   /** The second shape in the collision. */
   private FrameShape3DReadOnly shapeB;

   /** The key point on the shape A. */
   private final FramePoint3D pointOnA = new FramePoint3D();
   /** The surface normal at {@code pointOnA}. */
   private final FrameVector3D normalOnA = new FrameVector3D();

   /** The key point on the shape B. */
   private final FramePoint3D pointOnB = new FramePoint3D();
   /** The surface normal at {@code pointOnB}. */
   private final FrameVector3D normalOnB = new FrameVector3D();

   /**
    * Creates a new empty collision result.
    */
   public EuclidFrameShape3DCollisionResult()
   {
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
      signedDistance = distance;
   }

   /** {@inheritDoc} */
   @Override
   public void setFrameShapeA(FrameShape3DReadOnly shapeA)
   {
      this.shapeA = shapeA;
   }

   /** {@inheritDoc} */
   @Override
   public void setFrameShapeB(FrameShape3DReadOnly shapeB)
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
   public FrameShape3DReadOnly getShapeA()
   {
      return shapeA;
   }

   /** {@inheritDoc} */
   @Override
   public FrameShape3DReadOnly getShapeB()
   {
      return shapeB;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3D getPointOnA()
   {
      return pointOnA;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3D getNormalOnA()
   {
      return normalOnA;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3D getPointOnB()
   {
      return pointOnB;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3D getNormalOnB()
   {
      return normalOnB;
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
      if (object instanceof EuclidFrameShape3DCollisionResultReadOnly)
         return EuclidFrameShape3DCollisionResultBasics.super.equals((EuclidFrameShape3DCollisionResultReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this collision result as follows:<br>
    * When shapes are colliding:
    *
    * <pre>
    * Collision test result: colliding, depth: 0.539
    * Shape A: Box3D - shapeFrameA, location: ( 0.540,  0.110,  0.319 ) - locationFrameA, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * Shape B: Capsule3D - shapeFrameB, location: ( 0.540,  0.110,  0.319 ) - locationFrameB, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * </pre>
    *
    * When shapes are not colliding:
    *
    * <pre>
    * Collision test result: non-colliding, separating distance: 0.539
    * Shape A: Box3D - shapeFrameA, location: ( 0.540,  0.110,  0.319 ) - locationFrameA, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * Shape B: Capsule3D - shapeFrameB, location: ( 0.540,  0.110,  0.319 ) - locationFrameB, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * </pre>
    *
    * @return the {@code String} representing this collision result.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getEuclidFrameShape3DCollisionResultString(this);
   }
}
