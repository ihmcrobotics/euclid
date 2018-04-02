package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Read-only interface for a line 3D expressed in a given reference frame.
 * <p>
 * A line 3D represents an infinitely long line in the XY-plane and defined by a point and a
 * direction.
 * </p>
 * <p>
 * In addition to representing a {@link Line3DReadOnly}, a {@link ReferenceFrame} is associated to a
 * {@code FrameLine3DReadOnly}. This allows, for instance, to enforce, at runtime, that operations
 * on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameLine3DReadOnly} extends {@code Line3DReadOnly}, it is compatible with
 * methods only requiring {@code Line3DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameLine3DReadOnly}.
 * </p>
 */
public interface FrameLine3DReadOnly extends Line3DReadOnly, ReferenceFrameHolder
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getPoint();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getDirection();

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if either argument is not expressed in the same reference
    *            frame as this frame line 3D.
    */
   default void get(FixedFramePoint3DBasics pointToPack, FixedFrameVector3DBasics directionToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      checkReferenceFrameMatch(directionToPack);
      Line3DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void get(FramePoint3DBasics pointToPack, FrameVector3DBasics directionToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      directionToPack.setReferenceFrame(getReferenceFrame());
      Line3DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code directionToPack} is not expressed in the same
    *            reference frame as this frame line 3D.
    */
   default void get(Point3DBasics pointToPack, FixedFrameVector3DBasics directionToPack)
   {
      checkReferenceFrameMatch(directionToPack);
      Line3DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void get(Point3DBasics pointToPack, FrameVector3DBasics directionToPack)
   {
      directionToPack.setReferenceFrame(getReferenceFrame());
      Line3DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code pointToPack} is not expressed in the same
    *            reference frame as this frame line 3D.
    */
   default void get(FixedFramePoint3DBasics pointToPack, Vector3DBasics directionToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      Line3DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void get(FramePoint3DBasics pointToPack, Vector3DBasics directionToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      Line3DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Compares {@code this} with {@code other} to determine if the two lines are collinear.
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the lines are collinear, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean isCollinear(FrameLine3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Line3DReadOnly.super.isCollinear(other, epsilon);
   }

   /**
    * Compares {@code this} with {@code other} to determine if the two lines are collinear.
    *
    * @param other the line to compare to. Not modified.
    * @param angleEpsilon the tolerance of the comparison for angle.
    * @param distanceEpsilon the tolerance of the comparison for distance.
    * @return {@code true} if the lines are collinear, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean isCollinear(FrameLine3DReadOnly other, double angleEpsilon, double distanceEpsilon)
   {
      checkReferenceFrameMatch(other);
      return Line3DReadOnly.super.isCollinear(other, angleEpsilon, distanceEpsilon);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not expressed
    *            in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, Point3DBasics closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   default double closestPointsWith(Line3DReadOnly otherLine, FixedFramePoint3DBasics closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(closestPointOnThisLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   default double closestPointsWith(Line3DReadOnly otherLine, FramePoint3DBasics closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      closestPointOnThisLineToPack.setReferenceFrame(getReferenceFrame());
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code closestPointOnOtherLineToPack}
    *            are not expressed in the same reference frame.
    */
   default double closestPointsWith(Line3DReadOnly otherLine, Point3DBasics closestPointOnThisLineToPack, FixedFramePoint3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(closestPointOnOtherLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   default double closestPointsWith(Line3DReadOnly otherLine, Point3DBasics closestPointOnThisLineToPack, FramePoint3DBasics closestPointOnOtherLineToPack)
   {
      closestPointOnOtherLineToPack.setReferenceFrame(getReferenceFrame());
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code otherLine}, and
    *            {@code closestPointOnThisLineToPack} are not expressed in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, FixedFramePoint3DBasics closestPointOnThisLineToPack,
                                    Point3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      checkReferenceFrameMatch(closestPointOnThisLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not expressed
    *            in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, FramePoint3DBasics closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      closestPointOnThisLineToPack.setReferenceFrame(getReferenceFrame());
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code otherLine}, and
    *            {@code closestPointOnOtherLineToPack} are not expressed in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, Point3DBasics closestPointOnThisLineToPack,
                                    FixedFramePoint3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      checkReferenceFrameMatch(closestPointOnOtherLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not expressed
    *            in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, Point3DBasics closestPointOnThisLineToPack, FramePoint3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      closestPointOnOtherLineToPack.setReferenceFrame(getReferenceFrame());
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code otherLine},
    *            {@code closestPointOnThisLineToPack}, and {@code closestPointOnOtherLineToPack} are
    *            not expressed in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, FixedFramePoint3DBasics closestPointOnThisLineToPack,
                                    FixedFramePoint3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      checkReferenceFrameMatch(closestPointOnThisLineToPack);
      checkReferenceFrameMatch(closestPointOnOtherLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not expressed
    *            in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, FramePoint3DBasics closestPointOnThisLineToPack,
                                    FramePoint3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      closestPointOnThisLineToPack.setReferenceFrame(getReferenceFrame());
      closestPointOnOtherLineToPack.setReferenceFrame(getReferenceFrame());
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code closestPointOnThisLineToPack},
    *            and {@code closestPointOnOtherLineToPack} are not expressed in the same reference
    *            frame.
    */
   default double closestPointsWith(Line3DReadOnly otherLine, FixedFramePoint3DBasics closestPointOnThisLineToPack,
                                    FixedFramePoint3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(closestPointOnThisLineToPack);
      checkReferenceFrameMatch(closestPointOnOtherLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   default double closestPointsWith(Line3DReadOnly otherLine, FramePoint3DBasics closestPointOnThisLineToPack, FramePoint3DBasics closestPointOnOtherLineToPack)
   {
      closestPointOnThisLineToPack.setReferenceFrame(getReferenceFrame());
      closestPointOnOtherLineToPack.setReferenceFrame(getReferenceFrame());
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes the minimum distance between this line and {@code otherLine}.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the other line to compute the distance from. Not modified.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not expressed
    *            in the same reference frame.
    */
   default double distance(FrameLine3DReadOnly otherLine)
   {
      checkReferenceFrameMatch(otherLine);
      return Line3DReadOnly.super.distance(otherLine);
   }

   /**
    * Computes the minimum distance the given 3D point and this line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code direction.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code point} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from the line. Not modified.
    * @return the minimum distance between the 3D point and this 3D line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default double distance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line3DReadOnly.super.distance(point);
   }

   /**
    * Calculates the coordinates of the point 'p' given the parameter 't' as follows:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the point
    * p<sub>0</sub> defining this line.
    * </p>
    *
    * @param t the parameter used to calculate the point coordinates.
    * @param pointToPack the point in which the coordinates of 'p' are stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToPack} are not expressed
    *            in the same reference frame.
    */
   default void pointOnLineGivenParameter(double t, FixedFramePoint3DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      Line3DReadOnly.super.pointOnLineGivenParameter(t, pointToPack);
   }

   /**
    * Calculates the coordinates of the point 'p' given the parameter 't' as follows:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the point
    * p<sub>0</sub> defining this line.
    * </p>
    *
    * @param t the parameter used to calculate the point coordinates.
    * @param pointToPack the point in which the coordinates of 'p' are stored. Modified.
    */
   default void pointOnLineGivenParameter(double t, FramePoint3DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      Line3DReadOnly.super.pointOnLineGivenParameter(t, pointToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default FramePoint3D orthogonalProjectionCopy(FramePoint3DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return new FramePoint3D(getReferenceFrame(), Line3DReadOnly.super.orthogonalProjectionCopy(pointToProject));
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return Line3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code projectionToPack} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(projectionToPack);
      return Line3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return Line3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pointToProject}, and
    *            {@code projectionToPack} are not expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      checkReferenceFrameMatch(projectionToPack);
      return Line3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return Line3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Calculates the parameter 't' corresponding to the coordinates of the given {@code pointOnLine}
    * 'p' by solving the line equation:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the point
    * p<sub>0</sub> defining this line.
    * </p>
    *
    * @param pointOnLine the coordinates of the 'p' from which the parameter 't' is to be calculated.
    *           The point has to be on the line. Not modified.
    * @param epsilon the maximum distance allowed between the given point and this line. If the given
    *           point is at a distance less than {@code epsilon} from this line, it is considered as
    *           being located on this line.
    * @return the value of the parameter 't' corresponding to the given point.
    * @throws RuntimeException if the given point is located at a distance greater than {@code epsilon}
    *            from this line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default double parameterGivenPointOnLine(FramePoint3DReadOnly pointOnLine, double epsilon)
   {
      checkReferenceFrameMatch(pointOnLine);
      return Line3DReadOnly.super.parameterGivenPointOnLine(pointOnLine, epsilon);
   }

   /**
    * Tests if the given is located on this line.
    * <p>
    * More precisely, the point is assumed to be on this line if it is located at a distance less than
    * {@code epsilon} from it.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @param epsilon the tolerance used for this test.
    * @return {@code true} if the point is located on this line, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnLine(FramePoint3DReadOnly point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      return Line3DReadOnly.super.isPointOnLine(point, epsilon);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstPointOnLineToPack} are
    *            not expressed in the same reference frame.
    */
   default void getTwoPointsOnLine(FixedFramePoint3DBasics firstPointOnLineToPack, Point3DBasics secondPointOnLineToPack)
   {
      checkReferenceFrameMatch(firstPointOnLineToPack);
      Line3DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line. Modified.
    */
   default void getTwoPointsOnLine(FramePoint3DBasics firstPointOnLineToPack, Point3DBasics secondPointOnLineToPack)
   {
      firstPointOnLineToPack.setReferenceFrame(getReferenceFrame());
      Line3DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondPointOnLineToPack} are
    *            not expressed in the same reference frame.
    */
   default void getTwoPointsOnLine(Point3DBasics firstPointOnLineToPack, FixedFramePoint3DBasics secondPointOnLineToPack)
   {
      checkReferenceFrameMatch(secondPointOnLineToPack);
      Line3DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line. Modified.
    */
   default void getTwoPointsOnLine(Point3DBasics firstPointOnLineToPack, FramePoint3DBasics secondPointOnLineToPack)
   {
      secondPointOnLineToPack.setReferenceFrame(getReferenceFrame());
      Line3DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line. Modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstPointOnLineToPack}, and
    *            {@code secondPointOnLineToPack} are not expressed in the same reference frame.
    */
   default void getTwoPointsOnLine(FixedFramePoint3DBasics firstPointOnLineToPack, FixedFramePoint3DBasics secondPointOnLineToPack)
   {
      checkReferenceFrameMatch(firstPointOnLineToPack);
      checkReferenceFrameMatch(secondPointOnLineToPack);
      Line3DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line. Modified.
    */
   default void getTwoPointsOnLine(FramePoint3DBasics firstPointOnLineToPack, FramePoint3DBasics secondPointOnLineToPack)
   {
      firstPointOnLineToPack.setReferenceFrame(getReferenceFrame());
      secondPointOnLineToPack.setReferenceFrame(getReferenceFrame());
      Line3DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two lines are
    * physically the same but either the point or vector of each line is different. For instance, if
    * {@code this.point == other.point} and {@code this.direction == - other.direction}, the two lines
    * are physically the same but this method returns {@code false}.
    * <p>
    * If the two lines have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal and are expressed in the same reference frame,
    *         {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameLine3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return Line3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two lines are geometrically similar.
    * <p>
    * Two lines are considered geometrically equal is they are collinear, pointing toward the same or
    * opposite direction.
    * </p>
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two lines represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean geometricallyEquals(FrameLine3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Line3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this line 3D is exactly equal to {@code other}.
    * <p>
    * If the two lines have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other line 3D to compare against this. Not modified.
    * @return {@code true} if the two lines are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameLine3DReadOnly other)
   {
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Line3DReadOnly.super.equals(other);
   }
}