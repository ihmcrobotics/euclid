package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * Read-only interface for a 2D tuple expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Tuple2DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameTuple2DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple2DReadOnly} extends {@code Tuple2DReadOnly}, it is compatible with
 * methods only requiring {@code Tuple2DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameTuple2DReadOnly}.
 * </p>
 */
public interface FrameTuple2DReadOnly extends Tuple2DReadOnly, EuclidFrameGeometry
{
   /**
    * Calculates the norm of the difference between {@code this} and {@code other}.
    * <p>
    * |{@code this} - {@code other}| = &radic;[({@code this.x} - {@code other.x})<sup>2</sup> +
    * ({@code this.y} - {@code other.y})<sup>2</sup>]
    * </p>
    * 
    * @param other the other tuple to compare to. Not modified.
    * @return the norm squared of the difference.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default double differenceNorm(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Tuple2DReadOnly.super.differenceNorm(other);
   }

   /**
    * Calculates the norm squared of the difference between {@code this} and {@code other}.
    * <p>
    * |{@code this} - {@code other}|<sup>2</sup> = ({@code this.x} - {@code other.x})<sup>2</sup> +
    * ({@code this.y} - {@code other.y})<sup>2</sup>
    * </p>
    * 
    * @param other the other tuple to compare to. Not modified.
    * @return the norm squared of the difference.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default double differenceNormSquared(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Tuple2DReadOnly.super.differenceNormSquared(other);
   }

   /**
    * Calculates and returns the value of the dot product of this tuple with {@code other}.
    * <p>
    * For instance, the dot product of two tuples p and q is defined as: <br>
    * p . q = &sum;<sub>i=1:2</sub>(p<sub>i</sub> * q<sub>i</sub>)
    * </p>
    *
    * @param other the other tuple used for the dot product. Not modified.
    * @return the value of the dot product.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default double dot(FrameTuple2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Tuple2DReadOnly.super.dot(other);
   }

   /**
    * Gets a representative {@code String} of this tuple 2D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * (-0.675, -0.102 ) - worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameTuple2DString(format, this);
   }
}
