package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a box 3D expressed in a given reference frame.
 * <p>
 * A box 3D is represented by its size, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameBox3DReadOnly extends Box3DReadOnly, FrameShape3DReadOnly
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

   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this box.
    * </p>
    */
   @Override
   default FramePoint3DReadOnly getCentroid()
   {
      return getPosition();
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 box. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWith(FrameLine3DReadOnly line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 box. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    * @throws ReferenceFrameMismatchException if {@code line} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default int intersectionWith(FrameLine3DReadOnly line, FramePoint3DBasics firstIntersectionToPack, FramePoint3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 box. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    */
   default int intersectionWith(Line3DReadOnly line, FramePoint3DBasics firstIntersectionToPack, FramePoint3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 box. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWith(Line3DReadOnly line, FixedFramePoint3DBasics firstIntersectionToPack, FixedFramePoint3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 box. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWith(FrameLine3DReadOnly line, FixedFramePoint3DBasics firstIntersectionToPack, FixedFramePoint3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWith(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return Box3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    */
   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FramePoint3DBasics firstIntersectionToPack,
                                FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());

      return Box3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return Box3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    * @throws ReferenceFrameMismatchException if either {@code pointOnLine} or {@code lineDirection} is
    *                                         not expressed in the same reference frame as
    *                                         {@code this}.
    */
   default int intersectionWith(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, FramePoint3DBasics firstIntersectionToPack,
                                FramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Box3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWith(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                FixedFramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return Box3DReadOnly.super.intersectionWith(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxBox3D(this, destinationFrame, boundingBoxToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint3DBasics[] getVertices()
   {
      FramePoint3D[] vertices = new FramePoint3D[8];
      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new FramePoint3D());
      return vertices;
   }

   /**
    * Pack the coordinates in world of the 8 vertices of this box in the given array.
    *
    * @param verticesToPack the array in which the coordinates are stored. Modified.
    * @throws IllegalArgumentException if the length of the given array is different than 8.
    * @throws NullPointerException     if any of the 8 first elements of the given array is
    *                                  {@code null}.
    */
   default void getVertices(FramePoint3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 8)
         throw new IllegalArgumentException("Array is too small, has to be at least 8 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   /**
    * Pack the coordinates in world of the 8 vertices of this box in the given array.
    *
    * @param verticesToPack the array in which the coordinates are stored. Modified.
    * @throws IllegalArgumentException        if the length of the given array is different than 8.
    * @throws NullPointerException            if any of the 8 first elements of the given array is
    *                                         {@code null}.
    * @throws ReferenceFrameMismatchException if any of the array's elements is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void getVertices(FixedFramePoint3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 8)
         throw new IllegalArgumentException("Array is too small, has to be at least 8 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
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
    * Packs the world coordinates of one of this box vertices.
    *
    * @param vertexIndex  the index in [0, 7] of the vertex to pack.
    * @param vertexToPack point in which the coordinates of the vertex are stored. Modified.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 7].
    */
   default void getVertex(int vertexIndex, FramePoint3DBasics vertexToPack)
   {
      getVertex(vertexIndex, (Point3DBasics) vertexToPack);
      vertexToPack.setReferenceFrame(getReferenceFrame());
   }

   /**
    * Packs the world coordinates of one of this box vertices.
    *
    * @param vertexIndex  the index in [0, 7] of the vertex to pack.
    * @param vertexToPack point in which the coordinates of the vertex are stored. Modified.
    * @throws IndexOutOfBoundsException       if {@code vertexIndex} is not in [0, 7].
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void getVertex(int vertexIndex, FixedFramePoint3DBasics vertexToPack)
   {
      checkReferenceFrameMatch(vertexToPack);
      getVertex(vertexIndex, (Point3DBasics) vertexToPack);
   }

   /** {@inheritDoc} */
   @Override
   FixedFrameBox3DBasics copy();

   /**
    * Tests separately and on a per component basis if the pose and the size of this box and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    * <p>
    * If the two boxes have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the other box which pose and size is to be compared against this box pose and
    *                size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two boxes are equal component-wise and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameBox3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Box3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two boxes are geometrically similar.
    *
    * @param other   the box to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two boxes represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default boolean geometricallyEquals(FrameBox3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Box3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this box 3D is exactly equal to {@code other}.
    * <p>
    * If the two boxes have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other box 3D to compare against this. Not modified.
    * @return {@code true} if the two boxes are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameBox3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getPose().equals(other.getPose()) && getSize().equals(other.getSize());
   }
}
