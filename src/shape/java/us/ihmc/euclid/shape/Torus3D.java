package us.ihmc.euclid.shape;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.interfaces.Torus3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * {@code Torus3D} represents a torus in the XY-plane.
 * <p>
 * Shape description:
 * <ul>
 * <li>The origin is located at the centroid or center of the torus.
 * <li>The axis of revolution is the z-axis.
 * <li>The torus is defined by two radii: {@code tubeRadius} that represents the radius of the tube,
 * and {@code radius} that is the radius for the center of the torus to the center of the tube.
 * </ul>
 * </p>
 */
public class Torus3D implements Torus3DBasics, GeometryObject<Torus3D>
{
   private final Point3D position = new Point3D();
   private final Vector3D axis = new Vector3D(Axis.Z);

   /** It is the radius for the center of the torus to the center of the tube. */
   private double radius;
   /** Represents the radius of the tube */
   private double tubeRadius;

   /**
    * Creates a new torus 3D with a radius of {@code 1}, and tube radius of {@code 0.1}.
    */
   public Torus3D()
   {
      this(1.0, 0.1);
   }

   /**
    * Creates a new torus 3D identical to {@code other}.
    *
    * @param other the other torus to copy. Not modified.
    */
   public Torus3D(Torus3D other)
   {
      set(other);
   }

   /**
    * Creates a new torus 3D and initializes its radii.
    *
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS} or
    *            if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   public Torus3D(double radius, double tubeRadius)
   {
      setRadii(radius, tubeRadius);
   }

   /**
    * Creates a new torus 3D and initializes its pose and radii.
    *
    * @param pose the position and orientation of this torus. Not modified.
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS} or
    *            if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   public Torus3D(Point3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      set(position, axis, radius, tubeRadius);
   }

   /**
    * Copies the {@code other} torus data into {@code this}.
    *
    * @param other the other torus to copy. Not modified.
    */
   @Override
   public void set(Torus3D other)
   {
      Torus3DBasics.super.set(other);
   }

   /**
    * Sets the radii of this torus 3D.
    *
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS} or
    *            if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   public void setRadii(double radius, double tubeRadius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Torus3D cannot be negative: " + radius);
      if (tubeRadius < 0.0)
         throw new IllegalArgumentException("The tube radius of a Torus3D cannot be negative: " + tubeRadius);

      this.radius = radius;
      this.tubeRadius = tubeRadius;
   }

   /**
    * Gets the radius from the torus center to the tube center.
    *
    * @return this torus main radius.
    */
   @Override
   public double getRadius()
   {
      return radius;
   }

   /**
    * Gets the radius of the tube of this torus.
    *
    * @return the radius of the tube.
    */
   @Override
   public double getTubeRadius()
   {
      return tubeRadius;
   }

   @Override
   public Point3D getPosition()
   {
      return position;
   }

   @Override
   public Vector3D getAxis()
   {
      return axis;
   }

   /**
    * Tests separately and on a per component basis if the pose and the radii of this torus and
    * {@code other}'s pose and radii are equal to an {@code epsilon}.
    *
    * @param other the other torus which pose and radii is to be compared against this torus pose and
    *           radii. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two tori are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Torus3D other, double epsilon)
   {
      return Torus3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two tori are geometrically similar.
    * <p>
    * This method accounts for the multiple combinations of radii and rotations that generate identical
    * tori. For instance, two tori that are identical but one is rotated around its main axis are
    * considered geometrically equal.
    * </p>
    *
    * @param other the torus to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two tori represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Torus3D other, double epsilon)
   {
      return Torus3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getTorus3DString(this);
   }
}
