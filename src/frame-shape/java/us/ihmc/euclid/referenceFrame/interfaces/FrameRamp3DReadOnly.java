package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;

/**
 * Read-only interface for a ramp 3D expressed in a given frame.
 * <p>
 * A ramp represents a 3D shape with a triangular section in the XZ-plane. Shape description:
 * <ul>
 * <li>The slope face starts from {@code x=0.0}, {@code z=0.0} to end at {@code x=size.getX()},
 * {@code z=size.getZ()}.
 * <li>The bottom face is horizontal (XY-plane) at {@code z=0.0}.
 * <li>The rear face is vertical (YZ-plane) at {@code x=size.getX()}.
 * <li>The left face is vertical (XZ-plane) at {@code y=-size.getY()/2.0}.
 * <li>The right face is vertical (XZ-plane) at {@code y=size.getY()/2.0}.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameRamp3DReadOnly extends Ramp3DReadOnly, FrameShape3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getSize();

   /** {@inheritDoc} */
   @Override
   FrameShape3DPoseReadOnly getPose();

   /** {@inheritDoc} */
   @Override
   default FrameRotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      Ramp3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxRamp3D(this, destinationFrame, boundingBoxToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FrameVector3DBasics getRampSurfaceNormal()
   {
      FrameVector3D surfaceNormal = new FrameVector3D();
      getRampSurfaceNormal(surfaceNormal);
      return surfaceNormal;
   }

   /**
    * Computes and packs the surface normal of the slope face of this ramp.
    *
    * @param surfaceNormalToPack the surface normal of the slope. Modified.
    */
   default void getRampSurfaceNormal(FrameVector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.setReferenceFrame(getReferenceFrame());
      Ramp3DReadOnly.super.getRampSurfaceNormal(surfaceNormalToPack);
   }

   /**
    * Computes and packs the surface normal of the slope face of this ramp.
    *
    * @param surfaceNormalToPack the surface normal of the slope. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void getRampSurfaceNormal(FixedFrameVector3DBasics surfaceNormalToPack)
   {
      checkReferenceFrameMatch(surfaceNormalToPack);
      Ramp3DReadOnly.super.getRampSurfaceNormal(surfaceNormalToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint3DBasics[] getVertices()
   {
      FramePoint3D[] vertices = new FramePoint3D[6];
      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new FramePoint3D());
      return vertices;
   }

   /**
    * Pack the coordinates in world of the 6 vertices of this ramp in the given array.
    *
    * @param verticesToPack the array in which the coordinates are stored. Modified.
    * @throws IllegalArgumentException if the length of the given array is different than 6.
    * @throws NullPointerException     if any of the 6 first elements of the given array is
    *                                  {@code null}.
    */
   default void getVertices(FramePoint3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 6)
         throw new IllegalArgumentException("Array is too small, has to be at least 6 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   /**
    * Pack the coordinates in world of the 6 vertices of this ramp in the given array.
    *
    * @param verticesToPack the array in which the coordinates are stored. Modified.
    * @throws IllegalArgumentException        if the length of the given array is different than 6.
    * @throws NullPointerException            if any of the 6 first elements of the given array is
    *                                         {@code null}.
    * @throws ReferenceFrameMismatchException if any of the array's elements is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void getVertices(FixedFramePoint3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 6)
         throw new IllegalArgumentException("Array is too small, has to be at least 6 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint3DBasics getVertex(int vertexIndex)
   {
      FramePoint3D vertex = new FramePoint3D();
      getVertex(vertexIndex, vertex);
      return vertex;
   }

   /**
    * Packs one of this ramp vertices in the base coordinates.
    *
    * @param vertexIndex  the index in [0, 5] of the vertex to pack.
    * @param vertexToPack point in which the coordinates of the vertex are stored. Modified.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 5].
    */
   default void getVertex(int vertexIndex, FramePoint3DBasics vertexToPack)
   {
      vertexToPack.setReferenceFrame(getReferenceFrame());
      Ramp3DReadOnly.super.getVertex(vertexIndex, vertexToPack);
   }

   /**
    * Packs one of this ramp vertices in the base coordinates.
    *
    * @param vertexIndex  the index in [0, 5] of the vertex to pack.
    * @param vertexToPack point in which the coordinates of the vertex are stored. Modified.
    * @throws IndexOutOfBoundsException       if {@code vertexIndex} is not in [0, 5].
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void getVertex(int vertexIndex, FixedFramePoint3DBasics vertexToPack)
   {
      checkReferenceFrameMatch(vertexToPack);
      Ramp3DReadOnly.super.getVertex(vertexIndex, vertexToPack);
   }

   /** {@inheritDoc} */
   @Override
   FixedFrameRamp3DBasics copy();

   /**
    * Tests separately and on a per component basis if the pose and the size of this ramp and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    * <p>
    * If the two ramps have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the other ramp which pose and size is to be compared against this ramp pose and
    *                size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ramps are equal component-wise and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameRamp3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Ramp3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ramps are geometrically similar,
    * i.e. the difference between their size are less than or equal to {@code epsilon} and their poses
    * are geometrically similar given {@code epsilon}.
    *
    * @param other   the ramp to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ramps represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default boolean geometricallyEquals(FrameRamp3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Ramp3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this ramp 3D is exactly equal to {@code other}.
    * <p>
    * If the two ramps have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other ramp 3D to compare against this. Not modified.
    * @return {@code true} if the two ramps are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameRamp3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getPose().equals(other.getPose()) && getSize().equals(other.getSize());
   }
}
