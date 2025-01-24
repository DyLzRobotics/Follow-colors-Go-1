from telegram import Update
from telegram.ext import ApplicationBuilder, CommandHandler, ContextTypes

# Token del bot
TOKEN = "7900027175:AAGbnNQTsM5YKGv8krenhVz4XCwxki-bcMU"


# Lista de movimientos del robot
async def muve_robot(update: Update, context: ContextTypes.DEFAULT_TYPE):
    # Obtener argumentos del comando
    datos = context.args

    if not datos:
        await update.message.reply_text(
            "Por favor, especifica la parte del cuerpo, posición y acción que el robot debe realizar.\n"
            "Ejemplo: /Muve brazo izquierdo rotar"
        )
    else:
        parte = datos[0]
        posicion = datos[1] if len(datos) > 1 else "desconocida"
        accion = datos[2] if len(datos) > 2 else "desconocida"

        await update.message.reply_text(
            f"Has indicado los siguientes parámetros:\n"
            f"  - Parte del robot: {parte}\n"
            f"  - Posición: {posicion}\n"
            f"  - Acción: {accion}\n\n"
            f"Procesando solicitud..."
        )

    return datos


# Información general del robot
async def info_robot(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text(
        "Información del robot y cómo usar este bot:\n\n"
        "Comandos disponibles:\n"
        "  /Info - Información general sobre el robot.\n"
        "  /InfoMuve - Lista de las partes del robot y acciones posibles.\n"
        "  /Muve - Ejecuta un movimiento en una parte específica del robot.\n\n"
        "Detalles del robot:\n"
        "  - Motores de los brazos: 2 motores (25 kg de capacidad cada uno).\n"
        "  - Motores de las manos: 2 motores (2.5 kg de capacidad cada uno).\n"
        "  - Motores de la cabeza:\n"
        "      * Cuello y boca: 2 motores (25 kg cada uno).\n"
        "      * Ojos: 2 motores (2.5 kg cada uno).\n"
        "      * Detalles adicionales: 2 motores (9 g cada uno)."
    )


# Información de las partes y acciones del robot
async def infomuve_robot(update: Update, context: ContextTypes.DEFAULT_TYPE):
    print(context.args)
    await update.message.reply_text(
        "Partes y acciones disponibles para el robot:\n\n"
        "Partes del robot:\n"
        "  - brazo: adelante, atrás, rotar\n"
        "  - cabeza: rotar, mover\n"
        "  - mano: abrir, cerrar, girar\n"
        "  - codo: adelante, atrás, rotar\n"
        "  - hombro: adelante, atrás\n"
        "  - ojos: abrir, cerrar, izquierda, derecha\n"
        "  - boca: abrir, cerrar\n"
        "  - cuello: rotar\n"
        "  - cadera: rotar\n\n"
        "Ejemplo de comando: /Muve brazo rotar 90 grados"
    )


# Configuración principal del bot
def main():
    application = ApplicationBuilder().token(TOKEN).build()

    # Manejadores de comandos
    application.add_handler(CommandHandler("Muve", muve_robot))
    application.add_handler(CommandHandler("Info", info_robot))
    application.add_handler(CommandHandler("InfoMuve", infomuve_robot))
    from telegram.ext import MessageHandler, filters


    # Iniciar el bot
    application.run_polling()

if __name__ == "__main__":
    main()
