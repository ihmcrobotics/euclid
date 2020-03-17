package us.ihmc.euclid.referenceFrame.api;

import static us.ihmc.euclid.referenceFrame.api.MethodSignature.getMethodSimpleName;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.*;
import java.util.Map.Entry;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;

public class ReflectionBasedBuilders
{
   private final static Map<Class<?>, RandomFrameTypeBuilder> frameTypeBuilders = new HashMap<>();
   private final static Map<Class<?>, RandomFramelessTypeBuilder> framelessTypeBuilders = new HashMap<>();

   private final static Set<Class<?>> frameRandomGeneratorLibrary = new HashSet<>();
   private final static Set<Class<?>> framelessRandomGeneratorLibrary = new HashSet<>();

   public static void registerFrameRandomGeneratorClasses(Class<?>... classes)
   {
      frameRandomGeneratorLibrary.addAll(Arrays.asList(classes));
   }

   public static void registerFramelessRandomGeneratorClasses(Class<?>... classes)
   {
      framelessRandomGeneratorLibrary.addAll(Arrays.asList(classes));
   }

   public static void registerFrameTypeBuilderSmart(Class<?> frameType)
   {
      registerFrameTypeBuilder(frameType, searchFrameGenerator(frameType));
   }

   public static void registerFrameTypeBuilder(Class<?> frameType, RandomFrameTypeBuilder frameTypeBuilder)
   {
      frameTypeBuilders.put(frameType, frameTypeBuilder);
   }

   public static void registerFramelessTypeBuilderSmart(Class<?> framelessType)
   {
      registerFramelessTypeBuilder(framelessType, searchFramelessGenerator(framelessType));
   }

   public static void registerFramelessTypeBuilder(Class<?> framelessType, RandomFramelessTypeBuilder framelessTypeBuilder)
   {
      framelessTypeBuilders.put(framelessType, framelessTypeBuilder);
   }

   static Object newInstance(Class<?> type)
   {
      return newInstance(ReferenceFrame.getWorldFrame(), type);
   }

   static Object newInstance(ReferenceFrame frame, Class<?> type)
   {
      Object object = newInstanceOfFrameType(type, frame);
      if (object != null)
         return object;
      object = newInstanceOfFramelessType(type);
      if (object != null)
         return object;
      return newInstanceOfGenericType(type);
   }

   static Object[] newInstance(ReferenceFrame frame, Class<?>[] types)
   {
      Object[] instances = new Object[types.length];

      for (int i = 0; i < types.length; i++)
      {
         Class<?> type = types[i];

         if (type.isArray() && !type.getComponentType().isPrimitive())
         {
            Class<?> componentType = type.getComponentType();

            Object[] array = (Object[]) Array.newInstance(componentType, EuclidFrameAPITester.random.nextInt(15));

            for (int j = 0; j < array.length; j++)
            {
               array[j] = newInstance(frame, componentType);
               if (array[j] == null)
                  return null;
            }
            instances[i] = array;
         }
         else
         {
            instances[i] = newInstance(frame, type);
            if (instances[i] == null)
               return null;
         }
      }
      return instances;
   }

   private static Object newInstanceOfFramelessType(Class<?> type)
   {
      RandomFramelessTypeBuilder builder = null;
      Class<?> bestMatchingType = null;

      for (Entry<Class<?>, RandomFramelessTypeBuilder> entry : ReflectionBasedBuilders.framelessTypeBuilders.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(type))
            continue;

         if (bestMatchingType == null || bestMatchingType.isAssignableFrom(entry.getKey()))
         {
            bestMatchingType = entry.getKey();
            builder = entry.getValue();
         }
      }

      return builder == null ? null : builder.newInstance(EuclidFrameAPITester.random);
   }

   private static Object newInstanceOfFrameType(Class<?> type, ReferenceFrame referenceFrame)
   {
      RandomFrameTypeBuilder builder = null;
      Class<?> bestMatchingType = null;

      for (Entry<Class<?>, RandomFrameTypeBuilder> entry : ReflectionBasedBuilders.frameTypeBuilders.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(type))
            continue;

         if (bestMatchingType == null || bestMatchingType.isAssignableFrom(entry.getKey()))
         {
            bestMatchingType = entry.getKey();
            builder = entry.getValue();
         }
      }

      return builder == null ? null : builder.newInstance(EuclidFrameAPITester.random, referenceFrame);
   }

   private static Object newInstanceOfGenericType(Class<?> type)
   {
      if (type.isPrimitive())
      {
         if (type.equals(boolean.class))
            return EuclidFrameAPITester.random.nextBoolean();
         else if (type.equals(int.class) || type.equals(char.class) || type.equals(long.class))
            return EuclidFrameAPITester.random.nextInt(1000) - 500;
         else if (type.equals(float.class) || type.equals(double.class))
            return EuclidCoreRandomTools.nextDouble(EuclidFrameAPITester.random, 10.0);
         else
            return 0;
      }

      if (Transform.class.equals(type))
         return EuclidCoreRandomTools.nextAffineTransform(EuclidFrameAPITester.random);
      if (RigidBodyTransformReadOnly.class.isAssignableFrom(type))
         return EuclidCoreRandomTools.nextRigidBodyTransform(EuclidFrameAPITester.random);

      if (DenseMatrix64F.class.equals(type))
      {
         return RandomMatrices.createRandom(20, 20, EuclidFrameAPITester.random);
      }

      if (float[].class.equals(type))
      {
         float[] ret = new float[20];
         for (int i = 0; i < ret.length; i++)
            ret[i] = EuclidFrameAPITester.random.nextFloat();
         return ret;
      }

      if (double[].class.equals(type))
      {
         double[] ret = new double[20];
         for (int i = 0; i < ret.length; i++)
            ret[i] = EuclidFrameAPITester.random.nextDouble();
         return ret;
      }

      if (Collection.class.equals(type))
      {
         return null;
      }

      if (Object.class.equals(type))
      {
         return null;
      }

      try
      {
         return type.getConstructor().newInstance();
      }
      catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException
            | SecurityException e)
      {
         throw new RuntimeException("Could not instantiate an object of the type: " + type.getSimpleName() + " " + type);
      }
   }

   private static RandomFramelessTypeBuilder searchFramelessGenerator(Class<?> framelessMutableType)
   {
      List<Method> searchResult = new ArrayList<>();

      List<Method> allMethods = framelessRandomGeneratorLibrary.stream().flatMap(generatorClass -> Stream.of(generatorClass.getMethods()))
                                                               .collect(Collectors.toList());

      for (Method method : allMethods)
      {
         if (!Modifier.isStatic(method.getModifiers()) || !Modifier.isPublic(method.getModifiers()))
            continue;

         if (method.getParameterCount() != 1)
            continue;

         if (method.getParameterTypes()[0] != Random.class)
            continue;

         if (!framelessMutableType.isAssignableFrom(method.getReturnType()))
            continue;

         searchResult.add(method);
      }

      int nameLength = Integer.MAX_VALUE;
      Method randomGenerator = null;

      for (Method method : searchResult)
      {
         if (method.getName().length() < nameLength)
         {
            nameLength = method.getName().length();
            randomGenerator = method;
         }
      }
      System.out.println("Random generator for " + framelessMutableType.getSimpleName() + ", " + getMethodSimpleName(randomGenerator));

      Objects.requireNonNull(randomGenerator);
      final Method finalRandomGenerator = randomGenerator;

      return random ->
      {
         try
         {
            return finalRandomGenerator.invoke(null, random);
         }
         catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            throw new RuntimeException(e);
         }
      };
   }

   private static RandomFrameTypeBuilder searchFrameGenerator(Class<?> mutableFrameMutableType)
   {
      List<Method> searchResult = new ArrayList<>();

      List<Method> allMethods = frameRandomGeneratorLibrary.stream().flatMap(generatorClass -> Stream.of(generatorClass.getMethods()))
                                                           .collect(Collectors.toList());

      for (Method method : allMethods)
      {
         if (!Modifier.isStatic(method.getModifiers()) || !Modifier.isPublic(method.getModifiers()))
            continue;

         if (method.getParameterCount() != 2)
            continue;

         if (method.getParameterTypes()[0] != Random.class || method.getParameterTypes()[1] != ReferenceFrame.class)
            continue;

         if (!mutableFrameMutableType.isAssignableFrom(method.getReturnType()))
            continue;

         searchResult.add(method);
      }

      int nameLength = Integer.MAX_VALUE;
      Method randomGenerator = null;

      for (Method method : searchResult)
      {
         if (method.getName().length() < nameLength)
         {
            nameLength = method.getName().length();
            randomGenerator = method;
         }
      }

      Objects.requireNonNull(randomGenerator);

      System.out.println("Random generator for " + mutableFrameMutableType.getSimpleName() + ", " + getMethodSimpleName(randomGenerator));

      final Method finalRandomGenerator = randomGenerator;

      return (random, frame) ->
      {
         try
         {
            return (ReferenceFrameHolder) finalRandomGenerator.invoke(null, random, frame);
         }
         catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            throw new RuntimeException(e);
         }
      };
   }
}
