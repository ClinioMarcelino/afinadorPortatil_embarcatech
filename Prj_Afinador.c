#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "ws2818b.pio.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "kissfft/kiss_fft.h"
#include "kissfft/kiss_fftr.h"

// ====================================================================================================
//
//                                   DEFINIÇÕES
//
// ====================================================================================================

// Configurações
#define ADC_PICO 26
#define MIC_CHANNEL 2
#define MIC_PIN (ADC_PICO + MIC_CHANNEL)
#define SAMPLES 1024                  // Mais amostras = melhor resolução
#define SAMPLE_RATE 4000              // 4 kHz (ajuste conforme necessário)
#define AMPLIFICATION_FACTOR 10.0f    // Amplificação software (se necessário)
#define TOLERANCIA_AFINADOR 0.5f
#define LED_PIN 7
#define LED_COUNT 25

// I2C e Display
#define I2C_SDA 14
#define I2C_SCL 15
#define SSD1306_BUFFER_LENGTH (ssd1306_width * ssd1306_n_pages)

// Botões
#define BTN_B 6
#define BTN_A 5

// Configurações do joystick
#define JSK_X_CHANNEL 1
#define JSK_Y_CHANNEL 0
#define JSK_X_PIN (ADC_PICO + JSK_X_CHANNEL)
#define JSK_Y_PIN (ADC_PICO + JSK_X_CHANNEL)
#define JSK_BUTTON_PIN 22

#define MAX_FREQ 448.0f
#define MIN_FREQ 415.0f
#define REF_FREQ 440.0f

// ====================================================================================================
//
//                                   VARIAVEIS GLOBAIS
//
// ====================================================================================================

// Buffers
float samples[SAMPLES];
char str[20];
float effective_sample_rate = SAMPLE_RATE;

float referencia = REF_FREQ;

uint8_t luminosidade_display = 0;

// Estrutura para pixels
typedef struct pixel_t {
    uint8_t G, R, B;
} npLED_t;

// Variáveis globais
npLED_t leds[LED_COUNT];
PIO np_pio;
uint sm;
const uint8_t luminosidade[5] = {128,64,32,16,254}; // ordenado do menor para maior brilho

//const char* notas[] = { "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#" };

const uint8_t NOTE_PATTERNS[12][25] = {
    // A
    {0,1,1,1,0,
    0,1,0,1,0,
    0,1,1,1,0,
    0,1,0,1,0,
    0,1,0,1,0},
    // A#
    {0,1,1,1,1,
    0,1,0,1,0,
    0,1,1,1,0,
    0,1,0,1,0,
    0,1,0,1,0},
    // B
    {0,1,1,0,0,
    0,1,0,1,0,
    0,1,1,0,0,
    0,1,0,1,0,
    0,1,1,0,0},
    // C
    {0,1,1,1,0, 
    0,1,0,0,0, 
    0,1,0,0,0, 
    0,1,0,0,0, 
    0,1,1,1,0},
    // C#
    {0,1,1,1,1, 
     0,1,0,0,0, 
     0,1,0,0,0,
     0,1,0,0,0,
     0,1,1,1,0},
    // D
    {0,1,1,0,0,
    0,1,0,1,0,
    0,1,0,1,0,
    0,1,0,1,0,
    0,1,1,0,0},
    // D#
    {0,1,1,0,1,
    0,1,0,1,0,
    0,1,0,1,0,
    0,1,0,1,0,
    0,1,1,0,0},
    // E
    {0,1,1,1,0,
    0,1,0,0,0,
    0,1,1,1,0,
    0,1,0,0,0,
    0,1,1,1,0},
    // F
    {0,1,1,1,0,
    0,1,0,0,0,
    0,1,1,1,0,
    0,1,0,0,0,
    0,1,0,0,0},
    // F#
    {0,1,1,1,1,
    0,1,0,0,0,
    0,1,1,1,0,
    0,1,0,0,0,
    0,1,0,0,0},
    // G
    {0,1,1,1,0,
    0,1,0,0,0,
    0,1,0,1,0,
    0,1,0,1,0,
    0,1,1,1,0},
    // G#
    {0,1,1,1,1,
    0,1,0,0,0,
    0,1,0,1,0,
    0,1,0,1,0,
    0,1,1,1,0},
};

uint map(uint entrada, uint in_min, uint in_max, uint out_min, uint out_max){
    return (entrada - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ====================================================================================================
//
//                                   INICIALIZAÇÕES
//
// ====================================================================================================

void init_adc(){
    adc_init();       // Inicializa o ADC

    adc_gpio_init(MIC_PIN);
    adc_gpio_init(JSK_X_PIN);
    adc_gpio_init(JSK_Y_PIN);

    adc_select_input(MIC_CHANNEL);

}

void init_matriz_led(uint pin) {
    // Cria programa PIO.
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;

    // Toma posse de uma máquina PIO.
    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
    }

    // Inicia programa na máquina PIO obtida.
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

    // Limpa buffer de pixels.
    for (uint i = 0; i < LED_COUNT; ++i) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

void init_i2c(){
    // Inicialização do i2c
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();
}

void init_btn(uint pin){
    gpio_init(pin);
    gpio_set_dir(pin,GPIO_IN);
    gpio_pull_up(pin);
}


// ====================================================================================================
//
//                                   MÉTODOS MATRIZ LED
//
// ====================================================================================================

void set_matriz_led(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

void write_matriz_led() {
    // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
}

void clear_matriz_led() {
    for (uint i = 0; i < LED_COUNT; ++i)
        set_matriz_led(i, 0, 0, 0);

    sleep_ms(10);
}

void applyLedTransformations(uint8_t *pattern) {
    // Inverte colunas nas linhas ímpares
    for(int row = 5; row >=0; row = row-=2) {
        for(int col = 0; col < 2; col++) {
            int idx1 = row * 5 + col;
            int idx2 = row * 5 + (4 - col);
            uint8_t temp = pattern[idx1];
            pattern[idx1] = pattern[idx2];
            pattern[idx2] = temp;
        }
    }

    for(int i = 0; i < 12; i++) { 
        uint8_t temp = pattern[i];
        pattern[i] = pattern[24 - i];
        pattern[24 - i] = temp;
    }
}

void display_indicador_afinacao(float diff){
    // clear_matriz_esq();
    if(diff <= TOLERANCIA_AFINADOR && diff >= -TOLERANCIA_AFINADOR)
        set_matriz_led(14,0,luminosidade[luminosidade_display],0);
    
    
    if(diff > TOLERANCIA_AFINADOR)
        set_matriz_led(15,luminosidade[luminosidade_display],luminosidade[luminosidade_display],0);
    else if(diff < -TOLERANCIA_AFINADOR)
        set_matriz_led(5,luminosidade[luminosidade_display],luminosidade[luminosidade_display],0);

    if(diff > TOLERANCIA_AFINADOR*3)
        set_matriz_led(24,luminosidade[luminosidade_display],0,0);
    else if(diff < -TOLERANCIA_AFINADOR*3)
        set_matriz_led(4,luminosidade[luminosidade_display],0,0);
    
    write_matriz_led(np_pio);
}

void display_nota(uint8_t index, uint8_t intensidade, float diff) {
    clear_matriz_led();

    uint8_t transformed[25];
    memcpy(transformed, NOTE_PATTERNS[index], 25);
    applyLedTransformations(transformed);

    for(int i = 0; i < 25; i++) {
        if(transformed[i]) {
            set_matriz_led(i, luminosidade[intensidade], luminosidade[intensidade], luminosidade[intensidade]); 
            if(i==20)
                set_matriz_led(i, 0, 0, luminosidade[intensidade]); 
        }
        sleep_ms(10);
    }

    display_indicador_afinacao(diff);

    write_matriz_led(np_pio);
}

void alterar_luminosidade(uint8_t *ssd, struct render_area *frame_area){
    adc_select_input(JSK_Y_CHANNEL);
    
    // Limpa o display
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, frame_area);
    while(1){
        if(!gpio_get(BTN_A) || !gpio_get(BTN_B))
            break;


        
        uint aux = adc_read();
        aux = map(aux,0,4095,0,100);

        if(aux > 75)
            luminosidade_display++;
        
        if(aux < 25)
            luminosidade_display--;
        
        

        if(luminosidade_display > 4)
            luminosidade_display = 4;
        
        if (luminosidade_display < 0 )
            luminosidade_display = 0;

        sleep_ms(50);

        // Converte o contador para string
        snprintf(str, sizeof(str), "Luminosidade %d ", luminosidade_display+1);

        // Desenha a string do contador no display
        ssd1306_draw_string(ssd, 5, 20, str);

        // Renderiza o buffer no display
        render_on_display(ssd, frame_area);

        snprintf(str, sizeof(str), "B Voltar");

        // Desenha a string do contador no display
        ssd1306_draw_string(ssd, 5, 40, str);

        // Renderiza o buffer no display
        render_on_display(ssd, frame_area);

        set_matriz_led(12,luminosidade[luminosidade_display],luminosidade[luminosidade_display],luminosidade[luminosidade_display]);
        write_matriz_led();
        sleep_ms(250);
    }
    clear_matriz_led();
    write_matriz_led();
}


// ====================================================================================================
//
//                                   MÉTODOS CALCULOS FREQUENCIA
//
// ====================================================================================================

// Janela de Blackman-Nuttall para redução de leakage
void aplica_janela(float *sample) {
    for (int i = 0; i < SAMPLES; i++) {
        float a0 = 0.3635819;
        float a1 = 0.4891775;
        float a2 = 0.1365995;
        float a3 = 0.0106411;
        float w = a0 - a1 * cosf(2 * M_PI * i / (SAMPLES - 1)) 
                    + a2 * cosf(4 * M_PI * i / (SAMPLES - 1)) 
                    - a3 * cosf(6 * M_PI * i / (SAMPLES - 1));
        sample[i] *= w;
    }
}

// Coleta de áudio com medição da taxa efetiva
void coleta_audio(float *sample) {
    absolute_time_t t0 = get_absolute_time();
    uint32_t soma = 0;
    adc_select_input(MIC_CHANNEL);

    for (int i = 0; i < SAMPLES; i++) {
        sample[i] = (float) adc_read() - 2048.0f;  // Remove offset DC (ADC de 12 bits)
        sample[i] *= AMPLIFICATION_FACTOR;        // Amplificação em software
        soma += sample[i];
        sleep_us(1000000 / SAMPLE_RATE);
    }

    // Cálculo da taxa de amostragem efetiva
    absolute_time_t t1 = get_absolute_time();
    effective_sample_rate = SAMPLES / (absolute_time_diff_us(t0, t1) / 1e6f);
    
    // Aplica janela e remove DC residual
    aplica_janela(sample);
}

// Interpolação parabólica refinada
float refine_peak(int peak_index, kiss_fft_cpx *out) {
    if (peak_index <= 0 || peak_index >= SAMPLES / 2) return 0.0f;
    float y0 = sqrtf(out[peak_index-1].r * out[peak_index-1].r + out[peak_index-1].i * out[peak_index-1].i);
    float y1 = sqrtf(out[peak_index].r * out[peak_index].r + out[peak_index].i * out[peak_index].i);
    float y2 = sqrtf(out[peak_index+1].r * out[peak_index+1].r + out[peak_index+1].i * out[peak_index+1].i);
    float delta = 0.5f * (y0 - y2) / (y0 - 2*y1 + y2);
    return (peak_index + delta) * effective_sample_rate / SAMPLES;
}

// Cálculo da FFT e exibição
float calculo_frequencia() {
    kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);
    kiss_fft_cpx out[SAMPLES / 2 + 1];
    kiss_fftr(cfg, samples, out);
    free(cfg);

    int max_index = 0;
    float max_mag = 0;
    float threshold = 100.0f;  // Ignorar ruído de fundo

    for (int i = 1; i < SAMPLES / 2; i++) {
        float mag = sqrtf(out[i].r * out[i].r + out[i].i * out[i].i);
        if (mag > max_mag && mag > threshold) {
            max_mag = mag;
            max_index = i;
        }
    }

    return refine_peak(max_index, out);
}

/*
// Função para calcular a frequência usando Zero-Crossing
float calculate_frequency(uint16_t *buffer, uint32_t sample_rate) {
    uint32_t crossings = 0;
    for (int i = 1; i < BUFFER_SIZE; i++) {
        // Detecta cruzamentos para cima e para baixo
        if ((buffer[i - 1] < 2048 && buffer[i] >= 2048) || 
            (buffer[i - 1] >= 2048 && buffer[i] < 2048)) {
            crossings++;
        }
    }
    // Fórmula: (Número de cruzamentos / 2) * (Taxa de amostragem / Tamanho do buffer)
    float zero_cross = (crossings / 2.0f) * (sample_rate / (float)BUFFER_SIZE);
    float fft = calcula_fft();
    
    return (zero_cross+fft)/2;
}
*/

float identificar_nota(float freq_medida, float ref_freq, uint8_t *indice) {
    // Calcula a diferença em semitons
    float semitons = 12.0 * log2(freq_medida / ref_freq);

    // Arredonda para o semitom mais próximo
    int semitom_arredondado = round(semitons);

    // Determina o índice correto na escala (garante valor positivo 0-11)
    *indice = ((semitom_arredondado % 12) + 12) % 12;

    // Calcula a frequência da nota teórica mais próxima
    float nota_freq = ref_freq * powf(2.0, semitom_arredondado / 12.0);

    // Retorna a diferença em Hz (positivo = agudo, negativo = grave)
    return freq_medida - nota_freq;
}


// ====================================================================================================
//
//                                   MÉTODOS OLED
//
// ====================================================================================================

void write_menu_on_display(uint8_t num_tela, uint8_t* ssd) {
    static char *text_inicial[] = {
        "Selecione uma ",
        "opcao",
        "",
        "B Configurar",
        "",
        "A Afinar"
    };

    static char *text_page_2[] = {
        "",
        "B Next",
        "",
        "A Brilho"
    };

    static char *text_page_3[] = {
        "",
        "B Next",
        "",
        "A Frequencia",
        "  base"
    };

    static char *text_page_4[] = {
        "",
        "B Next",
        "",
        "A Menu"
    };

    char **text = NULL;
    uint8_t num_linhas = 0;

    switch (num_tela) {
        case 0:
            text = text_inicial;
            num_linhas = sizeof(text_inicial) / sizeof(text_inicial[0]);
            break;
        case 1:
            text = text_page_2;
            num_linhas = sizeof(text_page_2) / sizeof(text_page_2[0]);
            break;
        case 2:
            text = text_page_3;
            num_linhas = sizeof(text_page_3) / sizeof(text_page_3[0]);
            break;
        case 3:
            text = text_page_4;
            num_linhas = sizeof(text_page_4) / sizeof(text_page_4[0]);
            break;
        default:
            return;
    }

    memset(ssd, 0, ssd1306_buffer_length);

    int y = 0;
    for (uint i = 0; i < num_linhas; i++) {
        if (text[i] != NULL) {
            ssd1306_draw_string(ssd, 5, y, text[i]);
            y += 8;
        }
    }
}

void afinador(uint8_t *ssd, struct render_area *frame_area){
        while(true){
            if(!gpio_get(BTN_A) || !gpio_get(BTN_B) ){
                sleep_ms(200);
                clear_matriz_led();
                write_matriz_led();
                return;
            }

            coleta_audio(samples); 

            // Limpa o display
            memset(ssd, 0, ssd1306_buffer_length);

            // Calcula a frequência 
            float freq = calculo_frequencia();

            // Converte o contador para string
            snprintf(str, sizeof(str), "Freq %.0f Hz", freq);

            // Desenha a string do contador no display
            ssd1306_draw_string(ssd, 5, 20, str);

            // Renderiza o buffer no display
            render_on_display(ssd, frame_area);

            if (freq < 15.0){
                clear_matriz_led();
                write_matriz_led();
                continue;
            }

            uint8_t index;

            float diff = identificar_nota(freq, referencia, &index);

            
            display_nota(index,luminosidade_display,diff);
            
            

            // Aguarda 100 ms antes da próxima leitura
            sleep_ms(100);
        }
    }

void alterar_frequencia(uint8_t *ssd, struct render_area *frame_area){
    adc_select_input(JSK_Y_CHANNEL);
    
    // Limpa o display
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, frame_area);
    while(1){
        if(!gpio_get(BTN_A) || !gpio_get(BTN_B))
            break;

        uint aux = adc_read();
        aux = map(aux,0,4095,0,100);

        if (aux > 75)
            referencia += 1;
            if (referencia > MAX_FREQ)
                referencia = MAX_FREQ;
        
        if (aux < 25)
            referencia -= 1;
            if (referencia < MIN_FREQ)
                referencia = MIN_FREQ;


        // Converte o contador para string
        snprintf(str, sizeof(str), "Frequencia %.0f ", referencia);

        // Desenha a string do contador no display
        ssd1306_draw_string(ssd, 5, 20, str);

        // Renderiza o buffer no display
        render_on_display(ssd, frame_area);

        snprintf(str, sizeof(str), "B Voltar");

        // Desenha a string do contador no display
        ssd1306_draw_string(ssd, 5, 40, str);

        // Renderiza o buffer no display
        render_on_display(ssd, frame_area);
        
        sleep_ms(250);
    }
}

void menu_oled(uint8_t menu, uint8_t *ssd, struct render_area *frame_area) {
    write_menu_on_display(0, ssd);
    render_on_display(ssd, frame_area);

    while(1) {

        if (!gpio_get(BTN_A)) {
            sleep_ms(20); 
            if (!gpio_get(BTN_A)) { 
                while (!gpio_get(BTN_A)) {
                    sleep_ms(10);
                }
                sleep_ms(50);

                switch (menu) {
                    case 0:
                        afinador(ssd, frame_area);
                        break;
                    case 1:
                        alterar_luminosidade(ssd, frame_area);
                        break;
                    case 2:
                        alterar_frequencia(ssd,frame_area);
                        break;
                    case 3:
                        menu = 0;
                        break;
                    default:
                        break;
                }

                menu = 0;
                write_menu_on_display(0, ssd);
                render_on_display(ssd, frame_area);
            }
        }

        // Verifica BTN_B com debounce
        if (!gpio_get(BTN_B)) {
            sleep_ms(20);
            if (!gpio_get(BTN_B)) {
                while (!gpio_get(BTN_B)) {
                    sleep_ms(10);
                }
                sleep_ms(50);

                menu++;
                if (menu > 3)
                    menu = 0;
                write_menu_on_display(menu, ssd);
                render_on_display(ssd, frame_area);
            }
        }

        sleep_ms(50);
    }
}

int main(){
    stdio_init_all();

    init_adc();
    
    init_matriz_led(LED_PIN);
    clear_matriz_led();
    write_matriz_led();

    init_i2c();
    
    init_btn(BTN_B);
    init_btn(BTN_A);

        struct render_area frame_area = {
        start_column : 0,
        end_column : ssd1306_width - 1,
        start_page : 0,
        end_page : ssd1306_n_pages - 1
    };

    calculate_render_area_buffer_length(&frame_area);

    // zera o display inteiro
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    static uint menu = 0;

    // afinador(ssd,&frame_area);
    menu_oled(menu,ssd,&frame_area);

    return 0;
}
