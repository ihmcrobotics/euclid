package us.ihmc.euclid.shape.primitives.interfaces;

import java.util.ArrayDeque;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface IntermediateVariableSupplier
{
   Point3DBasics requestPoint3D();

   void releasePoint3D(Point3DBasics variableToRelease);

   Vector3DBasics requestVector3D();

   void releaseVector3D(Vector3DBasics variableToRelease);

   public static IntermediateVariableSupplier defaultIntermediateVariableSupplier()
   {
      return new IntermediateVariableSupplier()
      {
         @Override
         public Point3DBasics requestPoint3D()
         {
            return new Point3D();
         }

         @Override
         public void releasePoint3D(Point3DBasics variableToRelease)
         {
         }

         @Override
         public Vector3DBasics requestVector3D()
         {
            return new Vector3D();
         }

         @Override
         public void releaseVector3D(Vector3DBasics variableToRelease)
         {
         }
      };
   }

   public static IntermediateVariableSupplier garbageFreeIntermediateVariableSupplier()
   {
      return new IntermediateVariableSupplier()
      {
         private final ArrayDeque<Point3DBasics> point3DPool = new ArrayDeque<>();
         private final ArrayDeque<Vector3DBasics> vector3DPool = new ArrayDeque<>();

         @Override
         public Point3DBasics requestPoint3D()
         {
            if (point3DPool.isEmpty())
               return new Point3D();
            else
               return point3DPool.poll();
         }

         @Override
         public void releasePoint3D(Point3DBasics variableToRelease)
         {
            point3DPool.addLast(variableToRelease);
         }

         @Override
         public Vector3DBasics requestVector3D()
         {
            if (vector3DPool.isEmpty())
               return new Vector3D();
            else
               return vector3DPool.poll();
         }

         @Override
         public void releaseVector3D(Vector3DBasics variableToRelease)
         {
            vector3DPool.addLast(variableToRelease);
         }
      };
   }
}
