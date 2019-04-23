package us.ihmc.euclid.shape.primitives.interfaces;

import java.util.ArrayDeque;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Represents a supplier for managing intermediate variables used to stored an intermediate result
 * when performing multi-stage operations.
 * <p>
 * This interface offers two implementations:
 * <ul>
 * <li>{@link #defaultIntermediateVariableSupplier()} provides a guaranteed thread-safe variable
 * supplier, but generates garbage.
 * <li>{@link #garbageFreeIntermediateVariableSupplier()} provides a recycling variable supplier
 * which does not guarantee thread-safe operations.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface IntermediateVariableSupplier
{
   /**
    * Requests a point for storing an intermediate result immediately.
    *
    * @return an available point.
    */
   Point3DBasics requestPoint3D();

   /**
    * Release a previously used point that is not needed any longer.
    *
    * @param variableToRelease the no longer needed point.
    */
   void releasePoint3D(Point3DBasics variableToRelease);

   /**
    * Requests a vector for storing an intermediate result immediately.
    *
    * @return an available vector.
    */
   Vector3DBasics requestVector3D();

   /**
    * Release a previously used vector that is not needed any longer.
    *
    * @param variableToRelease the no longer needed vector.
    */
   void releaseVector3D(Vector3DBasics variableToRelease);

   /**
    * Creates a new variable supplier that guarantees thread-safe operations, but allocates memory at
    * runtime.
    *
    * @return the new variable supplier.
    */
   public static IntermediateVariableSupplier defaultIntermediateVariableSupplier()
   {
      return new IntermediateVariableSupplier()
      {
         /** {@inheritDoc} */
         @Override
         public Point3DBasics requestPoint3D()
         {
            return new Point3D();
         }

         /** {@inheritDoc} */
         @Override
         public void releasePoint3D(Point3DBasics variableToRelease)
         {
         }

         /** {@inheritDoc} */
         @Override
         public Vector3DBasics requestVector3D()
         {
            return new Vector3D();
         }

         /** {@inheritDoc} */
         @Override
         public void releaseVector3D(Vector3DBasics variableToRelease)
         {
         }
      };
   }

   /**
    * Creates a new variable supplier that recycles memory, but does not guarantees thread-safe
    * operations.
    *
    * @return the new variable supplier.
    */
   public static IntermediateVariableSupplier garbageFreeIntermediateVariableSupplier()
   {
      return new IntermediateVariableSupplier()
      {
         private final ArrayDeque<Point3DBasics> point3DPool = new ArrayDeque<>();
         private final ArrayDeque<Vector3DBasics> vector3DPool = new ArrayDeque<>();

         /** {@inheritDoc} */
         @Override
         public Point3DBasics requestPoint3D()
         {
            if (point3DPool.isEmpty())
               return new Point3D();
            else
               return point3DPool.poll();
         }

         /** {@inheritDoc} */
         @Override
         public void releasePoint3D(Point3DBasics variableToRelease)
         {
            point3DPool.addLast(variableToRelease);
         }

         /** {@inheritDoc} */
         @Override
         public Vector3DBasics requestVector3D()
         {
            if (vector3DPool.isEmpty())
               return new Vector3D();
            else
               return vector3DPool.poll();
         }

         /** {@inheritDoc} */
         @Override
         public void releaseVector3D(Vector3DBasics variableToRelease)
         {
            vector3DPool.addLast(variableToRelease);
         }
      };
   }
}
