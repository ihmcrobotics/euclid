package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a 3D axis-aligned bounding box expressed in a constant reference
 * frame, i.e. the reference frame of this object cannot be changed via this interface.
 */
public interface FixedFrameBoundingBox3DBasics extends BoundingBox3DBasics, FrameBoundingBox3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   FixedFramePoint3DBasics getMinPoint();

   /** {@inheritDoc} */
   @Override
   FixedFramePoint3DBasics getMaxPoint();

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param referenceFrame the reference frame in which the given {@code min} is expressed.
    * @param min            the minimum coordinate for this bounding box. Not modified.
    * @throws RuntimeException                if any of the minimum coordinates is strictly greater
    *                                         than the maximum coordinate on the same axis.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setMin(ReferenceFrame referenceFrame, Point3DReadOnly min)
   {
      checkReferenceFrameMatch(referenceFrame);
      setMin(min);
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param min the minimum coordinate for this bounding box. Not modified.
    * @throws RuntimeException                if any of the minimum coordinates is strictly greater
    *                                         than the maximum coordinate on the same axis.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setMin(FramePoint3DReadOnly min)
   {
      setMin(min.getReferenceFrame(), min);
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param referenceFrame the reference frame in which the given {@code max} is expressed.
    * @param max            the maximum coordinate for this bounding box. Not modified.
    * @throws RuntimeException                if any of the minimum coordinates is strictly greater
    *                                         than the maximum coordinate on the same axis.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setMax(ReferenceFrame referenceFrame, Point3DReadOnly max)
   {
      checkReferenceFrameMatch(referenceFrame);
      setMax(max);
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param max the maximum coordinate for this bounding box. Not modified.
    * @throws RuntimeException                if any of the minimum coordinates is strictly greater
    *                                         than the maximum coordinate on the same axis.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setMax(FramePoint3DReadOnly max)
   {
      setMax(max.getReferenceFrame(), max);
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param referenceFrame the reference frame in which the given {@code min} and {@code max} are
    *                       expressed.
    * @param min            the new minimum coordinates for this bounding box. Not modified.
    * @param max            the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException                if any of the minimum coordinates is strictly greater
    *                                         than the maximum coordinate on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly min, Point3DReadOnly max)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(min, max);
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param min the new minimum coordinates for this bounding box. Not modified.
    * @param max the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException                if any of the minimum coordinates is strictly greater
    *                                         than the maximum coordinate on the same axis.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      min.checkReferenceFrameMatch(max);
      set(min.getReferenceFrame(), min, max);
   }

   /**
    * Redefines this bounding box given its {@code center} location and half its size along each axis
    * {@code halfSize}.
    *
    * @param referenceFrame the reference frame in which the given {@code center} and {@code halfSize}
    *                       are expressed.
    * @param center         the new center location of this bounding box. Not modified.
    * @param halfSize       half the size of this bounding box. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly center, Vector3DReadOnly halfSize)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(center, halfSize);
   }

   /**
    * Redefines this bounding box given its {@code center} location and half its size along each axis
    * {@code halfSize}.
    *
    * @param center   the new center location of this bounding box. Not modified.
    * @param halfSize half the size of this bounding box. Not modified.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly center, Vector3DReadOnly halfSize)
   {
      set(center.getReferenceFrame(), center, halfSize);
   }

   /**
    * Redefines this bounding box given its {@code center} location and half its size along each axis
    * {@code halfSize}.
    *
    * @param center   the new center location of this bounding box. Not modified.
    * @param halfSize half the size of this bounding box. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly center, FrameVector3DReadOnly halfSize)
   {
      center.checkReferenceFrameMatch(halfSize);
      set(center.getReferenceFrame(), center, halfSize);
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param referenceFrame the reference frame in which {@code other} is expressed.
    * @param other          the bounding box used to redefine this bounding box. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, BoundingBox3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param other the bounding box used to redefine this bounding box. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameBoundingBox3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Combines this bounding box with {@code other} such that it becomes the smallest bounding box
    * containing this and {@code other}.
    * <p>
    * For each coordinates, if it is {@link Double#NaN} for one of the two bounding boxes, then the
    * coordinate of the other bounding box is used to update this bounding box. As a result, if for
    * instance {@code this} was set to {@link Double#NaN} beforehand, this operation would be
    * equivalent to {@code this.set(other}.
    * </p>
    *
    * @param referenceFrame the reference frame in which {@code other} is expressed.
    * @param other          the other bounding box to combine with this. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void combine(ReferenceFrame referenceFrame, BoundingBox3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      combine(other);
   }

   /**
    * Combines this bounding box with {@code other} such that it becomes the smallest bounding box
    * containing this and {@code other}.
    * <p>
    * For each coordinates, if it is {@link Double#NaN} for one of the two bounding boxes, then the
    * coordinate of the other bounding box is used to update this bounding box. As a result, if for
    * instance {@code this} was set to {@link Double#NaN} beforehand, this operation would be
    * equivalent to {@code this.set(other}.
    * </p>
    *
    * @param other the other bounding box to combine with this. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void combine(FrameBoundingBox3DReadOnly other)
   {
      combine(other.getReferenceFrame(), other);
   }

   /**
    * Sets this bounding box to the union of {@code boundingBoxOne} and {@code boundingBoxTwo}.
    * <p>
    * This bounding box is set such that it is the smallest bounding box containing the two given
    * bounding boxes.
    * </p>
    * <p>
    * For each coordinates, if it is {@link Double#NaN} for one of the two bounding boxes, then the
    * coordinate of the other bounding box is used to update this bounding box. As a result, if for
    * instance {@code boundingBoxOne} was set to {@link Double#NaN} beforehand, this operation would be
    * equivalent to {@code this.set(boundingBoxTwo}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the given {@code boundingBoxOne} and
    *                       {@code boundingBoxTwo} are expressed.
    * @param boundingBoxOne the first bounding box. Can be the same instance as this. Not modified.
    * @param boundingBoxTwo the second bounding box. Can be the same instance as this. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void combine(ReferenceFrame referenceFrame, BoundingBox3DReadOnly boundingBoxOne, BoundingBox3DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(referenceFrame);
      combine(boundingBoxOne, boundingBoxTwo);
   }

   /**
    * Sets this bounding box to the union of {@code boundingBoxOne} and {@code boundingBoxTwo}.
    * <p>
    * This bounding box is set such that it is the smallest bounding box containing the two given
    * bounding boxes.
    * </p>
    * <p>
    * For each coordinates, if it is {@link Double#NaN} for one of the two bounding boxes, then the
    * coordinate of the other bounding box is used to update this bounding box. As a result, if for
    * instance {@code boundingBoxOne} was set to {@link Double#NaN} beforehand, this operation would be
    * equivalent to {@code this.set(boundingBoxTwo}.
    * </p>
    *
    * @param boundingBoxOne the first bounding box. Can be the same instance as this. Not modified.
    * @param boundingBoxTwo the second bounding box. Can be the same instance as this. Not modified.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void combine(FrameBoundingBox3DReadOnly boundingBoxOne, BoundingBox3DReadOnly boundingBoxTwo)
   {
      combine(boundingBoxOne.getReferenceFrame(), boundingBoxOne, boundingBoxTwo);
   }

   /**
    * Sets this bounding box to the union of {@code boundingBoxOne} and {@code boundingBoxTwo}.
    * <p>
    * This bounding box is set such that it is the smallest bounding box containing the two given
    * bounding boxes.
    * </p>
    * <p>
    * For each coordinates, if it is {@link Double#NaN} for one of the two bounding boxes, then the
    * coordinate of the other bounding box is used to update this bounding box. As a result, if for
    * instance {@code boundingBoxOne} was set to {@link Double#NaN} beforehand, this operation would be
    * equivalent to {@code this.set(boundingBoxTwo}.
    * </p>
    *
    * @param boundingBoxOne the first bounding box. Can be the same instance as this. Not modified.
    * @param boundingBoxTwo the second bounding box. Can be the same instance as this. Not modified.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void combine(BoundingBox3DReadOnly boundingBoxOne, FrameBoundingBox3DReadOnly boundingBoxTwo)
   {
      combine(boundingBoxTwo.getReferenceFrame(), boundingBoxOne, boundingBoxTwo);
   }

   /**
    * Sets this bounding box to the union of {@code boundingBoxOne} and {@code boundingBoxTwo}.
    * <p>
    * This bounding box is set such that it is the smallest bounding box containing the two given
    * bounding boxes.
    * </p>
    * <p>
    * For each coordinates, if it is {@link Double#NaN} for one of the two bounding boxes, then the
    * coordinate of the other bounding box is used to update this bounding box. As a result, if for
    * instance {@code boundingBoxOne} was set to {@link Double#NaN} beforehand, this operation would be
    * equivalent to {@code this.set(boundingBoxTwo}.
    * </p>
    *
    * @param boundingBoxOne the first bounding box. Can be the same instance as this. Not modified.
    * @param boundingBoxTwo the second bounding box. Can be the same instance as this. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void combine(FrameBoundingBox3DReadOnly boundingBoxOne, FrameBoundingBox3DReadOnly boundingBoxTwo)
   {
      boundingBoxOne.checkReferenceFrameMatch(boundingBoxTwo);
      combine(boundingBoxOne.getReferenceFrame(), boundingBoxOne, boundingBoxTwo);
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the supplied
    * points.
    *
    * @param referenceFrame   the reference frame in which the given {@code vertex3DSupplier} is
    *                         expressed.
    * @param vertex3DSupplier the supply of points.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void updateToIncludePoints(ReferenceFrame referenceFrame, Vertex3DSupplier vertex3DSupplier)
   {
      checkReferenceFrameMatch(referenceFrame);
      updateToIncludePoints(vertex3DSupplier);
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the supplied
    * points.
    *
    * @param vertex3DSupplier the supply of points.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void updateToIncludePoints(FrameVertex3DSupplier vertex3DSupplier)
   {
      updateToIncludePoints(vertex3DSupplier.getReferenceFrame(), vertex3DSupplier);
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param referenceFrame the reference frame in which the given {@code point} is expressed.
    * @param point          the point to be included in this bounding box. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void updateToIncludePoint(ReferenceFrame referenceFrame, Point3DReadOnly point)
   {
      checkReferenceFrameMatch(referenceFrame);
      updateToIncludePoint(point);
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param point the point to be included in this bounding box. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void updateToIncludePoint(FramePoint3DReadOnly point)
   {
      updateToIncludePoint(point.getReferenceFrame(), point);
   }
}
