import matplotlib.pyplot as plt

# Substitua as listas pelos valores fornecidos
q0 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
acuracia = [91, 97, 99, 100, 100, 100, 99, 99, 97]

# Plotar o gráfico com estilo
plt.plot(q0, acuracia, marker='o', linestyle='-', color='b', label='Acurácia')

# Adicionar grade
plt.grid(True, linestyle='--', alpha=0.7)

# Adicionar rótulos e título ao gráfico
plt.xlabel('q0', fontsize=12)
plt.ylabel('Acurácia (%)', fontsize=12)
plt.title('Gráfico de Acurácia em Função de q0', fontsize=14)

# Adicionar legenda
plt.legend()

# Adicionar limite nos eixos para melhor visualização
plt.xlim(0.1, 0.9)
plt.ylim(90, 105)

# Exibir o gráfico
plt.show()
